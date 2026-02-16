package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

@TeleOp(name = "Jules Master Controller", group = "Jules")
public class JulesMasterController extends OpMode {

    private JulesRobot robot;
    private String lastCommand = "none";
    private long lastCommandTime = 0;
    private String lastExecResult = "idle";

    @Override
    public void init() {
        robot = new JulesRobot(this);
        robot.init();

        // Show motor init status
        telemetry.addData("JULES", "Master Controller Initialized");
        telemetry.addData("Motors",
                (robot.leftFront != null ? "LF✓ " : "LF✗ ") +
                        (robot.leftRear != null ? "LR✓ " : "LR✗ ") +
                        (robot.rightFront != null ? "RF✓ " : "RF✗ ") +
                        (robot.rightRear != null ? "RR✓ " : "RR✗ "));
        telemetry.update();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        robot.update();

        // Peek at pending command for telemetry
        String pending = org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand.peekCommand();
        if (pending != null) {
            lastCommand = pending.length() > 80 ? pending.substring(0, 80) + "..." : pending;
            lastCommandTime = System.currentTimeMillis();

            // Show if it looks like JSON
            boolean isJson = pending.trim().startsWith("{");
            lastExecResult = isJson ? "JSON->executeCommand" : "text->legacy";
        }

        // Process commands
        robot.processCommands();

        // Auto-stop motors when duration expires
        robot.checkAutoStop();

        // Telemetry
        telemetry.addData("Status", "Running");
        telemetry.addData("Motors",
                (robot.leftFront != null ? "LF✓ " : "LF✗ ") +
                        (robot.leftRear != null ? "LR✓ " : "LR✗ ") +
                        (robot.rightFront != null ? "RF✓ " : "RF✗ ") +
                        (robot.rightRear != null ? "RR✓ " : "RR✗ "));
        telemetry.addData("Last Cmd", lastCommand);
        long ago = (lastCommandTime > 0) ? (System.currentTimeMillis() - lastCommandTime) / 1000 : -1;
        telemetry.addData("Cmd Age", ago >= 0 ? ago + "s ago" : "never");
        telemetry.addData("Exec Path", lastExecResult);
        telemetry.update();

        // Broadcast to App
        com.google.gson.JsonObject telem = robot.getTelemetryData();
        telem.addProperty("type", "telemetry");
        telem.addProperty("ts", System.currentTimeMillis());
        telem.addProperty("last_cmd", lastCommand);
        telem.addProperty("last_exec", lastExecResult);
        robot.publish(telem.toString());
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
