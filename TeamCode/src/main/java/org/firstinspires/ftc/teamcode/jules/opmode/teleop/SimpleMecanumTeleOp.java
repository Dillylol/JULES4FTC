package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

/**
 * A simple, bare-bones mecanum drive TeleOp.
 * Uses the JulesRobot abstraction for hardware mapping.
 */
@TeleOp(name = "Simple Mecanum TeleOp", group = "JULES")
public class SimpleMecanumTeleOp extends OpMode {

    private JulesRobot robot;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        // Mecanum drive control
        // Left stick for translation (y, x), Right stick for rotation (rx)
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing (Inverted per user request)
        double rx = -gamepad1.right_stick_x; // (Inverted per user request)

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.setDrivePowers(frontLeftPower, backLeftPower, frontRightPower, backRightPower);

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Powers", "fl:%.2f bl:%.2f fr:%.2f br:%.2f",
                frontLeftPower, backLeftPower, frontRightPower, backRightPower);

        if (robot.batterySensor != null) {
            telemetry.addData("Battery", "%.2f V", robot.batterySensor.getVoltage());
        }

        // --- JULES App Integration ---
        // 1. Update internal systems (e.g. Pedro Pathing)
        robot.update();

        // 2. Execute commands received from the App
        robot.processCommands();

        // 3. Send telemetry back to the App
        // We wrap the standard telemetry data in a JSON object
        com.google.gson.JsonObject telem = robot.getTelemetryData();
        telem.addProperty("type", "telemetry");
        telem.addProperty("ts", System.currentTimeMillis());
        // Add specific TeleOp data if needed
        telem.addProperty("opmode", "SimpleMecanumTeleOp");
        robot.publish(telem.toString());
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
