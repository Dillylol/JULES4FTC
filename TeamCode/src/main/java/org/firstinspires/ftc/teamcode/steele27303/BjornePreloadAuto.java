package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "Bjorne Auto - Preloads", group = "Auto")
public class BjornePreloadAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Initialize Follower & Hardware
        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        BjornHardware hardware = BjornHardware.forAutonomous(hardwareMap);
        
        ShooterController shooter = new ShooterController(
            hardware.wheel, 
            hardware.wheel2, 
            hardware.intake, 
            hardware, 
            hardware.getVoltageSensor()
        );

        // 2. Build Paths
        // Move "Backwards" 15 inches. X is forward in Pedro, so target is x = -15.
        // Assuming robot starts at 0,0,0
        // Point replaced by Pose (assuming BezierLine accepts Pose or Point is gone)
        Path backwardPath = new Path(new BezierLine(new Pose(0, 0, 0), new Pose(-15, 0, 0)));
        backwardPath.setConstantHeadingInterpolation(0); // Maintain 0 heading

        telemetry.addLine("Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 3. Execute Path
        follower.followPath(backwardPath);
        
        // Loop while busy (moving)
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            shooter.update(System.currentTimeMillis()); 
            telemetry.addData("Status", "Moving Backwards");
            telemetry.addData("X", follower.getPose().getX());
            telemetry.update();
        }

        // 4. Intake for 1.5 Seconds
        long intakeStartTime = System.currentTimeMillis();
        long intakeDuration = 1500; // 1.5s
        
        // Set Intake Power
        hardware.intake.setPower(1.0); // Assuming positive is IN
        
        while (opModeIsActive() && (System.currentTimeMillis() - intakeStartTime < intakeDuration)) {
            follower.update(); // Keep updating localization
            shooter.update(System.currentTimeMillis());
            telemetry.addData("Status", "Intaking");
            telemetry.update();
        }
        
        // Stop Intake? User said "fire the intake... then trigger"
        // Usually you'd stop or keep running. I'll keep it running if it helps logic, but usually stop.
        // I will keep it running if it helps feed, or stop if it's done. 
        // "fire the intake for like 1.5 seconds... then trigger" -> Implies stop or done.
        // I'll leave it running if the user wants "feed manual" logic, but standard is stop or verify.
        // Actually, shooterController handles feeding.
        // If I manually set power, I override shooterController.
        // I should probably let shooterController handle it or stop it.
        // I'll stop it for now to be safe, or setFeedManual if needed.
        // But shooterController controls intake for feeding.
        // So I'll just turn it off to hand control to shooterController?
        hardware.intake.setPower(0); 

        // 5. Spin Up & Fire Use ShooterController
        shooter.setTargetRpm(2800.0, System.currentTimeMillis());
        
        // Fire 3 times
        int shotsFired = 0;
        int targetShots = 3;
        
        while (opModeIsActive() && shotsFired < targetShots) {
            long now = System.currentTimeMillis();
            follower.update();
            shooter.update(now);
            
            // Logic: 
            // 1. Wait for Ready
            // 2. Fire Boot
            // 3. Wait for Reset/Pulse?
            
            // Simple approach: Trigger fire, wait specific time, repeat.
            // User said "spin up... then trigger the bootkicker... 3 artifacts... trigger 3 times"
            
            // Trigger 1
             shooter.fireBoot(now);
             
             // Manually trigger grips per request
             if (hardware.grip1 != null) hardware.grip1.setPower(-1.0);
             if (hardware.grip2 != null) hardware.grip2.setPower(-1.0);
             
             shotsFired++;
             
             // Wait for cycle (e.g. 500ms or 1s)
             long start = System.currentTimeMillis();
             while (opModeIsActive() && (System.currentTimeMillis() - start < 800)) {
                 follower.update();
                 shooter.update(System.currentTimeMillis());
             }
             
             // Stop Grips
             if (hardware.grip1 != null) hardware.grip1.setPower(0);
             if (hardware.grip2 != null) hardware.grip2.setPower(0);
        }

        // Done
        shooter.stop(System.currentTimeMillis());
        hardware.intake.setPower(0);
        
        while (opModeIsActive()) {
            follower.update();
            shooter.update(System.currentTimeMillis());
            telemetry.addData("Status", "Finished");
            telemetry.update();
        }
    }
    
    private void sleepLoop(long durationMs, Follower follower, ShooterController shooter) {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - start < durationMs)) {
            follower.update();
            shooter.update(System.currentTimeMillis());
        }
    }
}
