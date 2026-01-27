package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "VL53L1X Regression", group = "Sensor")
public class VL53L1XRegression extends LinearOpMode {

    private VL53L1X sensor;
    private int targetInches = 1;
    private List<Point> dataPoints = new ArrayList<>();

    // Smoothing
    private int lastMm = -1;

    private static class Point {
        double realMm;
        double measuredMm;

        public Point(double r, double m) {
            realMm = r;
            measuredMm = m;
        }
    }

    @Override
    public void runOpMode() {
        try {
            sensor = hardwareMap.get(VL53L1X.class, "vl53l1x");
            // Ensure no hardware offset is interfering with our software regression logic
            sensor.setCalibrationOffset(0);
            // Ensure no previous software params
            sensor.setCorrectionParams(1.0, 0.0);
        } catch (Exception e) {
            telemetry.addData("Error", "Sensor not found!");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "D-Pad Left/Right: Select Distance (1-10 in)");
        telemetry.addData("Controls", "A: Record Point");
        telemetry.addData("Controls", "B: Reset All");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Input
            if (gamepad1.dpad_right) {
                if (targetInches < 10)
                    targetInches++;
                sleep(200);
            } else if (gamepad1.dpad_left) {
                if (targetInches > 1)
                    targetInches--;
                sleep(200);
            }

            // Read Raw (using getRawDistanceMm to bypass any existing software correction)
            int currentMm = sensor.getRawDistanceMm();
            if (currentMm != -1) {
                lastMm = currentMm;
            }

            // Record
            if (gamepad1.a && lastMm != -1) {
                double realMm = targetInches * 25.4;
                dataPoints.add(new Point(realMm, lastMm));
                sleep(300);
            }

            // Reset
            if (gamepad1.b) {
                dataPoints.clear();
                sleep(300);
            }

            // Calculate Regression
            double alpha = 1.0;
            double beta = 0.0;
            String status = "Need 2+ Points";

            if (dataPoints.size() >= 2) {
                // Simple Linear Regression: Real = alpha * Measured + beta
                double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
                int n = dataPoints.size();

                for (Point p : dataPoints) {
                    sumX += p.measuredMm;
                    sumY += p.realMm;
                    sumXY += (p.measuredMm * p.realMm);
                    sumX2 += (p.measuredMm * p.measuredMm);
                }

                double denominator = (n * sumX2 - sumX * sumX);
                if (Math.abs(denominator) > 0.0001) {
                    alpha = (n * sumXY - sumX * sumY) / denominator;
                    beta = (sumY - alpha * sumX) / n;
                    status = String.format("y = %.4fx + %.2f", alpha, beta);

                    // Live Test of current parameters
                    // Apply to current reading
                    double correctedMm = lastMm * alpha + beta;
                    double error = Math.abs(correctedMm - (targetInches * 25.4));
                    telemetry.addData("Validation",
                            String.format("Inches: %.2f (Err: %.2f mm)", correctedMm / 25.4, error));
                }
            }

            // UI
            telemetry.addData("TARGET", targetInches + " INCHES");
            telemetry.addData("RAW READ", lastMm == -1 ? "WAITING..." : lastMm + " mm");
            telemetry.addData("Recorded Points", dataPoints.size());
            telemetry.addData("----------------", "-");
            telemetry.addData("REGRESSION", status);
            telemetry.addData("Alpha (Slope)", String.format("%.5f", alpha));
            telemetry.addData("Beta (Offset)", String.format("%.2f", beta));
            telemetry.addData("----------------", "-");
            telemetry.addData("CODE:", String.format("sensor.setCorrectionParams(%.5f, %.2f);", alpha, beta));

            telemetry.update();
        }
    }
}
