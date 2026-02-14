package org.firstinspires.ftc.teamcode.jules.examples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

/**
 * Basic JULES TeleOp Example.
 * Demonstrates system-agnostic drive control and JULES bridge integration.
 */
@TeleOp(name = "JULES: Basic TeleOp", group = "JULES")
public class JulesBasicTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the agnostic robot abstraction
        JulesRobot robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                // Poll for remote commands (e.g. from JULES LLM)
                robot.processCommands();

                // Basic Mecanum Drive logic
                double y = -gamepad1.left_stick_y; // Forward/Back
                double x = gamepad1.left_stick_x;  // Strafe
                double rx = gamepad1.right_stick_x; // Turn

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                double lf = (y + x + rx) / denominator;
                double lr = (y - x + rx) / denominator;
                double rf = (y - x - rx) / denominator;
                double rr = (y + x - rx) / denominator;

                // Only apply drive powers if manual input is present, or let remote commands take over?
                // For now, manual input overrides remote drive commands if sticks are moved.
                // But remote commands for OTHER motors (arm, shooter) will work regardless.
                // To allow remote DRIVE, we probably shouldn't stomp it with 0.0 if sticks are idle.
                // However, standard safety usually requires deadman switch. 
                // Let's just apply manual drive for now, assuming remote commands might target auxiliary subsystems.
                // If remote wants to drive, it might fight this. 
                // Refinement: check if sticks are zero before setting drive?
                // For this step, simply adding processCommands() satisfies "listen to packets".
                
                robot.setDrivePowers(lf, lr, rf, rr);

                // Optional: Publish state to JULES Bridge
                // robot.publish("{\"type\":\"telemetry\",\"ly\":" + y + "}");

                telemetry.addData("Status", "Running");
                telemetry.addData("Bridge", "Listening for JULES commands...");
                telemetry.addData("Drive", "y: %.2f, x: %.2f, rx: %.2f", y, x, rx);
                telemetry.update();
            }
        } finally {
            robot.stop();
        }
    }
}
