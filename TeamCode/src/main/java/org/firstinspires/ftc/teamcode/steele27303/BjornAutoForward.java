package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

@Autonomous(name = "Bjorn Auto - Forward (24)", group = "Auto")
public class BjornAutoForward extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Follower
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Build Path: Move to (24, 0)
        Path forwardPath = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)));
        forwardPath.setConstantHeadingInterpolation(0); 

        telemetry.addLine("Initialized. Forward (24).");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Execute Path
        follower.followPath(forwardPath);
        
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Status", "Moving Forward");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.update();
        }
        
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Status", "Finished");
            telemetry.update();
        }
    }
}
