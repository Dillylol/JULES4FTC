package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VL53L1X Distance Test", group = "Sensor")
public class VL53L1XDistanceTest extends OpMode {

    private VL53L1X sensor;

    @Override
    public void init() {
        try {
            sensor = hardwareMap.get(VL53L1X.class, "vl53l1x");
            telemetry.addData("Status", "Initialized. Measurements starting...");
            // Now startContinuous is called inside initSensor(), so we don't need to call
            // it manually.
        } catch (Exception e) {
            telemetry.addData("Error", "Check config: " + e.getMessage());
        }
    }

    private int lastDistance = -1;
    private int validReadings = 0;
    private long lastTime = System.currentTimeMillis();

    @Override
    public void loop() {
        if (sensor != null) {
            // Mode Select
            if (gamepad1.a) {
                sensor.setDistanceMode(VL53L1X.DistanceMode.Short);
                telemetry.addData("Mode Set", "SHORT");
            } else if (gamepad1.b) {
                sensor.setDistanceMode(VL53L1X.DistanceMode.Long);
                telemetry.addData("Mode Set", "LONG");
            } else if (gamepad1.x) {
                sensor.setDistanceMode(VL53L1X.DistanceMode.Medium);
                telemetry.addData("Mode Set", "MEDIUM");
            }

            int newDistance = sensor.getDistanceMm();

            // NOTE: getDistanceMm() returns -1 if the sensor isn't ready yet
            // (non-blocking).
            // We only update our display when we get a valid reading.
            if (newDistance != -1) {
                lastDistance = newDistance;
                validReadings++;
            }

            telemetry.addData("Status", newDistance != -1 ? "Reading..." : "Waiting for cycle...");

            if (lastDistance == -1) {
                telemetry.addData("Distance", "Waiting for first reading...");
            } else if (lastDistance == 65535) {
                telemetry.addData("Distance", "Out of Range");
            } else {
                telemetry.addData("Distance (mm)", lastDistance);
                telemetry.addData("Distance (in)", String.format("%.2f", lastDistance / 25.4));
            }

            // Show data rate
            long now = System.currentTimeMillis();
            if (now - lastTime > 1000) {
                telemetry.addData("Rate", validReadings + " Hz");
                validReadings = 0;
                lastTime = now;
            } else {
                telemetry.addData("Rate", "Calc...");
            }

            telemetry.addData("ID", String.format("%04X", sensor.getModelID()));
            telemetry.addData("Controls", "A=Short, B=Long, X=Med");
        }
        telemetry.update();
    }
}
