package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Diagnostic: Pathing Verification", group = "Diagnostic")
public class PathingDiagnosticAuto extends LinearOpMode {

    private Follower follower;

    private enum PathMode {
        FORWARD_24,
        STRAFE_RIGHT_24,
        DIAGONAL_24,
        SQUARE
    }

    private PathMode selectedMode = PathMode.FORWARD_24;
    private boolean dpadUpPrev, dpadDownPrev;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        // Init Loop for Selection
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_down && !dpadDownPrev) {
                int next = (selectedMode.ordinal() + 1) % PathMode.values().length;
                selectedMode = PathMode.values()[next];
            }
            if (gamepad1.dpad_up && !dpadUpPrev) {
                int prev = selectedMode.ordinal() - 1;
                if (prev < 0)
                    prev = PathMode.values().length - 1;
                selectedMode = PathMode.values()[prev];
            }
            dpadUpPrev = gamepad1.dpad_up;
            dpadDownPrev = gamepad1.dpad_down;

            telemetry.addLine("Select Path Mode (DPAD U/D):");
            telemetry.addData(">>", selectedMode);
            telemetry.update();
        }

        if (isStopRequested())
            return;

        // Build Path based on selection
        Path path = null;
        switch (selectedMode) {
            case FORWARD_24:
                // Forward 24 inches (X+)
                path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)));
                break;
            case STRAFE_RIGHT_24:
                // Right 24 inches (Y- or Y+ depending on config, usually Y- for Blue Right?)
                // Let's assume -Y is Right.
                path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(0, -24, 0)));
                break;
            case DIAGONAL_24:
                path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(24, -24, 0)));
                break;
            case SQUARE:
                // TODO: Implement PathChain for square
                // For now, just Forward
                path = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(24, 0, 0)));
                break;
        }

        if (path != null) {
            path.setConstantHeadingInterpolation(0);
            follower.followPath(path);
        }

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Mode", selectedMode);
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("H", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
