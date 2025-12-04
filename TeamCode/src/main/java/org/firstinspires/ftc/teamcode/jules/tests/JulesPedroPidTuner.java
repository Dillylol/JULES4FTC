package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonObject;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

@Autonomous(name = "JULES: Pedro PID Tuner", group = "JULES")
public class JulesPedroPidTuner extends OpMode {

    private JulesRobot robot;
    private boolean pathStarted = false;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        telemetry.addLine("Ready to Tune Pedro.");
    }

    @Override
    public void start() {
        robot.start();
        if (robot.follower != null) {
            // Simple 48 inch forward move
            robot.follower.followPath(
                robot.follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(0, 0, 0), new Pose(48, 0, 0)))
                    .setLinearHeadingInterpolation(0, 0)
                    .build(),
                true
            );
            pathStarted = true;
        }
    }

    @Override
    public void loop() {
        robot.update();
        
        if (robot.follower != null && pathStarted) {
            Pose current = robot.follower.getPose();
            // We want to log error. Pedro calculates this internally but we can just log Pose vs Target (approx)
            // Ideally we'd log the follower's internal error if accessible.
            // For now, let's log the Pose and let the Client compute error against the known path (0,0 -> 48,0).
            
            JsonObject data = new JsonObject();
            data.addProperty("type", "pedro_data");
            data.addProperty("x", current.getX());
            data.addProperty("y", current.getY());
            data.addProperty("h", current.getHeading());
            data.addProperty("busy", robot.follower.isBusy());
            robot.publish(data.toString());
            
            if (!robot.follower.isBusy()) {
                pathStarted = false;
                telemetry.addLine("Path Complete");
            }
        }
        
        telemetry.update();
    }
}
