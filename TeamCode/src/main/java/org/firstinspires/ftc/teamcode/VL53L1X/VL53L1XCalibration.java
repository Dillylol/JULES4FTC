package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VL53L1X Calibration", group = "Sensor")
public class VL53L1XCalibration extends LinearOpMode {

    private VL53L1X sensor;
    private double targetDistanceInches = 4.0; // Default target 4 inches
    private int currentOffset = 0;

    // Smoothing / Persistence variables
    private int lastMm = -1;
    private int validReadings = 0;
    private long lastTime = System.currentTimeMillis();

    @Override
    public void runOpMode() {
        // Initialize sensor
        try {
            sensor = hardwareMap.get(VL53L1X.class, "vl53l1x");
            currentOffset = sensor.getCalibrationOffset();
        } catch (Exception e) {
            telemetry.addData("Error", "Sensor not found!");
            telemetry.update();
            waitForStart();
            return;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "Use D-Pad Up/Down to set Target (Inches)");
        telemetry.addData("Instructions", "Press A to Calibrate");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 1. Handle Input (Target in Inches)
            if (gamepad1.dpad_up) {
                targetDistanceInches += 0.1;
                sleep(150);
            } else if (gamepad1.dpad_down) {
                targetDistanceInches -= 0.1;
                sleep(150);
            }

            if (gamepad1.x) { // coarser adjustment
                targetDistanceInches += 1.0;
                sleep(150);
            } else if (gamepad1.y) {
                targetDistanceInches -= 1.0;
                sleep(150);
            }

            // 2. Read Sensor (Smooth Data)
            int measuredMm = sensor.getDistanceMm();
            if (measuredMm != -1) {
                lastMm = measuredMm;
                validReadings++;
            }
            // Use lastMm for calculations to keep display stable

            // 3. Calculate suggested offset
            // Target (mm) = Inches * 25.4
            int targetMm = (int) Math.round(targetDistanceInches * 25.4);

            // NewOffset = CurrentOffset + (Target - Measured)
            // Use lastMm carefully - if -1 (never read), can't calibrate yet.
            int suggestedOffset = currentOffset;
            if (lastMm != -1) {
                suggestedOffset = currentOffset + (targetMm - lastMm);
            }

            // 4. Apply Calibration
            if (gamepad1.a && lastMm != -1) {
                currentOffset = suggestedOffset;
                sensor.setCalibrationOffset(currentOffset);
                sleep(200);
            }

            // 5. Reset to Zero
            if (gamepad1.b) {
                currentOffset = 0;
                sensor.setCalibrationOffset(0);
                sleep(200);
            }

            // 6. Rate Calculation
            long now = System.currentTimeMillis();
            String rateStr = "";
            if (now - lastTime > 1000) {
                rateStr = validReadings + " Hz";
                validReadings = 0;
                lastTime = now;
            }

            // Display
            telemetry.addData("TARGET", String.format("%.1f in  (%d mm)", targetDistanceInches, targetMm));

            if (lastMm != -1) {
                telemetry.addData("MEASURED", String.format("%.2f in  (%d mm)", lastMm / 25.4, lastMm));
                telemetry.addData("Offset (Current)", currentOffset + " mm");
                telemetry.addData("Offset (Suggested)", suggestedOffset + " mm");

                int error = Math.abs(lastMm - targetMm);
                if (error < 3) {
                    telemetry.addData("Status", "ALIGNED (Diff: " + error + "mm)");
                } else {
                    telemetry.addData("Status", "MISMATCH (Diff: " + (lastMm - targetMm) + "mm)");
                }
            } else {
                telemetry.addData("MEASURED", "Waiting for data...");
            }

            telemetry.addData("Rate", rateStr);
            telemetry.addData("----------------", "-");
            telemetry.addData("SAVE THIS LINE:", "sensor.setCalibrationOffset(" + currentOffset + ");");

            telemetry.update();
        }
    }
}
