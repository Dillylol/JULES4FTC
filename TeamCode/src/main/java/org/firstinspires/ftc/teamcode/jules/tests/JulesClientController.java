package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

@TeleOp(name = "JULES: Client Controller", group = "JULES")
public class JulesClientController extends OpMode {

    private JulesRobot robot;
    private final Queue<String> cmdQueue = new ConcurrentLinkedQueue<>();
    private JulesStreamBus.Subscription sub;
    private Thread listener;

    // Control State
    private double driveForward = 0;
    private double driveStrafe = 0;
    private double driveTurn = 0;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        
        // Setup Listener
        if (robot.streamBus != null) {
            try {
                sub = robot.streamBus.subscribe();
                listener = new Thread(() -> {
                    while (!Thread.currentThread().isInterrupted()) {
                        try {
                            String line = sub.take();
                            if (line == null) break;
                            cmdQueue.offer(line);
                        } catch (Exception e) { break; }
                    }
                });
                listener.start();
            } catch (Exception e) {
                telemetry.addData("Bus Error", e.getMessage());
            }
        }
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
        processCommands();

        // Apply Drive
        if (robot.follower != null) {
            robot.follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, false);
        } else {
            // Manual Mecanum math if no Pedro
            double y = -driveForward; // Remember, Y is reversed on gamepads usually
            double x = driveStrafe * 1.1; // Counteract imperfect strafing
            double rx = driveTurn;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double fl = (y + x + rx) / denominator;
            double bl = (y - x + rx) / denominator;
            double fr = (y - x - rx) / denominator;
            double br = (y + x - rx) / denominator;

            robot.setDrivePowers(fl, bl, fr, br);
        }
        
        telemetry.addData("Fwd", driveForward);
        telemetry.addData("Str", driveStrafe);
        telemetry.addData("Turn", driveTurn);
        telemetry.update();
    }

    private void processCommands() {
        String raw;
        while ((raw = cmdQueue.poll()) != null) {
            try {
                JsonElement el = GsonCompat.parse(raw);
                if (el.isJsonObject()) {
                    JsonObject obj = el.getAsJsonObject();
                    if (obj.has("cmd")) {
                        String cmd = obj.get("cmd").getAsString();
                        if (cmd.equals("drive")) {
                            driveForward = obj.has("y") ? obj.get("y").getAsDouble() : 0;
                            driveStrafe = obj.has("x") ? obj.get("x").getAsDouble() : 0;
                            driveTurn = obj.has("turn") ? obj.get("turn").getAsDouble() : 0;
                        } else if (cmd.equals("stop")) {
                            driveForward = 0;
                            driveStrafe = 0;
                            driveTurn = 0;
                        }
                    }
                }
            } catch (Exception e) {}
        }
    }

    @Override
    public void stop() {
        if (listener != null) listener.interrupt();
        if (sub != null) {
            try { sub.close(); } catch (Exception e) {}
        }
    }
}
