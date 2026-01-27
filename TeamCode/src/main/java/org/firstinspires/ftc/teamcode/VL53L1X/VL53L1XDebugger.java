package org.firstinspires.ftc.teamcode.VL53L1X;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "VL53L1X Register Debugger", group = "Sensor")
public class VL53L1XDebugger extends OpMode {

    private VL53L1X sensor;
    private int currentRegister = 0x010F; // Start at Model ID
    private boolean lastPadUp = false;
    private boolean lastPadDown = false;
    private boolean lastPadRight = false;
    private boolean lastPadLeft = false;

    @Override
    public void init() {
        try {
            // Retrieve via the custom class, not raw I2cDeviceSynch
            sensor = hardwareMap.get(VL53L1X.class, "vl53l1x");
            telemetry.addData("Status", "Initialized. Use D-Pad to change register.");
        } catch (Exception e) {
            telemetry.addData("Error", "Init fail: " + e.getMessage());
        }
    }

    @Override
    public void loop() {
        if (sensor == null)
            return;

        // Input Handling
        if (gamepad1.dpad_up && !lastPadUp)
            currentRegister++;
        if (gamepad1.dpad_down && !lastPadDown)
            currentRegister--;
        if (gamepad1.dpad_right && !lastPadRight)
            currentRegister += 16;
        if (gamepad1.dpad_left && !lastPadLeft)
            currentRegister -= 16;

        lastPadUp = gamepad1.dpad_up;
        lastPadDown = gamepad1.dpad_down;
        lastPadRight = gamepad1.dpad_right;
        lastPadLeft = gamepad1.dpad_left;

        // Clamp
        if (currentRegister < 0)
            currentRegister = 0;
        if (currentRegister > 0xFFFF)
            currentRegister = 0xFFFF;

        // Reading
        // Note: Generic read might fail if device is busy/disconnected, catch it?
        int byteVal = 0;
        int shortVal = 0;
        try {
            byteVal = sensor.readReg16Byte(currentRegister) & 0xFF;
            shortVal = sensor.readReg16Short(currentRegister) & 0xFFFF;
        } catch (Exception e) {
            telemetry.addData("Read Error", e.getMessage());
        }

        // Display
        telemetry.addData("TARGET REGISTER", String.format("0x%04X", currentRegister));
        telemetry.addData("----------------", "----");
        telemetry.addData("Value (8-bit)", "Hex: 0x%02X | Dec: %d", byteVal, byteVal);
        telemetry.addData("Value (16-bit)", "Hex: 0x%04X | Dec: %d", shortVal, shortVal);
        telemetry.addData("----------------", "----");
        telemetry.addData("Known Registers", "");
        telemetry.addData("0x010F", "Model ID (Expect 0xEA)");

        telemetry.update();
    }
}
