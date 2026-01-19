package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

@Autonomous(name = "Bjorn Auto - Diagonal BLUE (-15, -15)", group = "Auto")
public class BjornAutoDiagonalBlue extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Follower
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Build Path: Move to (-15, -15)
        Path diagonalPath = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(-15, -15, 0)));
        diagonalPath.setConstantHeadingInterpolation(0); 

        telemetry.addLine("Initialized. Diagonal BLUE (-15, -15).");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // Execute Path
        follower.followPath(diagonalPath);
        
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addData("Status", "Moving Diagonally");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.update();
        }
        
        // Wait at end
        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Status", "Finished");
            telemetry.update();
        }
    }
}
