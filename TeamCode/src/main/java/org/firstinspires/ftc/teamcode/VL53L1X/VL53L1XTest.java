package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "VL53L1X Test", group = "Sensor")
public class VL53L1XTest extends OpMode {

    private VL53L1X sensor;

    @Override
    public void init() {
        try {
            // Since the sensor is configured as "VL53L1X Time of Flight" in the Robot
            // Config,
            // the SDK automatically creates the VL53L1X instance.
            // We simply retrieve it by its class.
            sensor = hardwareMap.get(VL53L1X.class, "vl53l1x");

            // Initialization is handled by the SDK via the I2cDeviceType annotation logic,
            // but we can call initialize() if we want to be safe or reset it.
            // sensor.initialize(); // SDK calls doInitialize() automatically on retrieving?
            // Verification needed.
            // Usually for I2cDeviceSynchDevice we might need to verify arming.

            telemetry.addData("Status", "Found VL53L1X Driver instance");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find 'vl53l1x' configuration.");
            telemetry.addData("Detail", e.getMessage());
            telemetry.addData("Tip", "Make sure config name is 'vl53l1x' and type is 'VL53L1X Time of Flight'");
        }
    }

    @Override
    public void loop() {
        if (sensor != null) {
            boolean connected = sensor.isConnected();
            int modelId = 0;
            if (connected || true) {
                modelId = sensor.getModelID();
            }

            telemetry.addData("Connection", connected ? "ESTABLISHED" : "FAILED (Check ID)");
            telemetry.addData("Model ID (Hex)", String.format("0x%04X", modelId));
            telemetry.addData("Expected ID", "0xEACC (approx)");
        } else {
            telemetry.addData("Status", "Waiting for init...");
        }
        telemetry.update();
    }
}
