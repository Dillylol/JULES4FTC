package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

@Autonomous(name = "JULES: Client Auto", group = "JULES")
public class JulesClientAuto extends OpMode {

    private JulesRobot robot;
    private final Queue<String> cmdQueue = new ConcurrentLinkedQueue<>();
    private JulesStreamBus.Subscription sub;
    private Thread listener;
    
    private boolean isMoving = false;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        
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
        telemetry.addLine("Waiting for Client Waypoints...");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();
        processCommands();
        
        if (robot.follower != null) {
            if (isMoving && !robot.follower.isBusy()) {
                isMoving = false;
                robot.publish("{\"type\":\"event\",\"payload\":\"arrived\"}");
            }
            
            Pose p = robot.follower.getPose();
            telemetry.addData("Pose", String.format("%.1f, %.1f, %.1f", p.getX(), p.getY(), Math.toDegrees(p.getHeading())));
        }
        telemetry.addData("Status", isMoving ? "Moving" : "Idle");
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
                        if (cmd.equals("goto")) {
                            if (robot.follower == null) {
                                telemetry.addLine("Error: No Pedro Follower");
                                continue;
                            }
                            double x = obj.get("x").getAsDouble();
                            double y = obj.get("y").getAsDouble();
                            double h = obj.has("h") ? Math.toRadians(obj.get("h").getAsDouble()) : 0;
                            
                            Pose current = robot.follower.getPose();
                            Pose target = new Pose(x, y, h);
                            
                            robot.follower.followPath(
                                robot.follower.pathBuilder()
                                    .addPath(new BezierLine(new Pose(current.getX(), current.getY(), 0), new Pose(x, y, 0)))
                                    .setLinearHeadingInterpolation(current.getHeading(), h)
                                    .build(),
                                true
                            );
                            isMoving = true;
                            telemetry.addLine("Going to " + x + ", " + y);
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
