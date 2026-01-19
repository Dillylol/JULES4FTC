package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

@TeleOp(name = "BjornTeleOp - BLUE", group = "TeleOp")
public class BjornTeleOpBlue extends BaseBjornTeleOp {
    @Override
    protected int getTargetTagId() {
        return CameraConfig.BLUE_GOAL_TAG_ID;
    }

    @Override
    protected String getAllianceName() {
        return "BLUE";
    }

    @Override
    public void runOpMode() {
        initCommon();
        
        waitForStart();
        

        boolean triggerPrev = false;
        
        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            
            // --- 1. Systems Update ---
            hardware.updateEstimator(); // Encoder Prediction
            
            // Camera Update
            TagObservation targetTag = null;
            List<TagObservation> dets = camera.pollDetections();
            for (TagObservation obs : dets) {
                if (obs.id == getTargetTagId()) {
                    targetTag = obs;
                    break;
                }
            }
            camera.publishDetections();
            
            // --- 2. Input Handling ---
            
            // Drive
            double y = -gamepad1.left_stick_y * DRIVE_SPEED; 
            double x = gamepad1.left_stick_x * DRIVE_SPEED * 1.1; 
            double rx = gamepad1.right_stick_x * TURN_SPEED;
            
            calculateDrive(x, y, rx);

            // IMU Reset
            if (gamepad1.dpad_down) {
                hardware.imu.resetYaw();
            }

            // Turret Mode Toggle (D-Pad Up Edge) - G1 OR G2
            boolean dpadUp = gamepad1.dpad_up;
            if (dpadUp && !dpadUpPrev) {
                turretActiveMode = !turretActiveMode;
            }
            dpadUpPrev = dpadUp;
            
            boolean dpadUp2 = gamepad2.dpad_up;
            if (dpadUp2 && !dpadUp2Prev) {
                turretActiveMode = !turretActiveMode;
            }
            dpadUp2Prev = dpadUp2;
            
            // SIMPLIFIED TURRET LOGIC
            double currentTicks = hardware.turret.getCurrentPosition();
            double currentDeg = currentTicks / BjornHardware.TURRET_TICKS_PER_DEGREE;
            
            // 2. Determine Power
            double turretPower = 0.0;
            if (turretActiveMode && targetTag != null) {
                // Tracking
                double error = -targetTag.yaw;
                turretPower = error * TURRET_KP;
                
            } else {
                // Manual (G1 OR G2)
                boolean left = gamepad1.dpad_left || gamepad2.dpad_left;
                boolean right = gamepad1.dpad_right || gamepad2.dpad_right;

                if (left) turretPower = -TURRET_MANUAL_POWER;
                else if (right) turretPower = TURRET_MANUAL_POWER;
                else turretPower = 0.0;
            }
            
            // 3. Apply Hard Limits
            if (currentDeg > TURRET_LIMIT_DEG && turretPower > 0) turretPower = 0;
            else if (currentDeg < -TURRET_LIMIT_DEG && turretPower < 0) turretPower = 0;
            
            if (hardware.turret != null) hardware.turret.setPower(turretPower);

            // Intake (G1 Only per plan, but user said "do not alter g1 stuff")
            double intakePower = 0.0;
            if (gamepad1.a) intakePower = 1.0;
            else if (gamepad1.x) intakePower = -1.0;
            hardware.intake.setPower(intakePower);

            // Shooter Ramp - G1 OR G2
            boolean b = gamepad1.b;
            if (b && !bPrev) shooterActive = !shooterActive;
            bPrev = b;

            boolean b2 = gamepad2.b;
            if (b2 && !b2Prev) shooterActive = !shooterActive;
            b2Prev = b2;
            
            // Shooter Idle - G2 Only
            boolean y2 = gamepad2.y;
            if (y2 && !y2Prev) shooterIdle = !shooterIdle;
            y2Prev = y2;
            
            // Allow Idle to auto-enable if we turn OFF shooter active? No, simple toggle.
            // Priority: Active > Idle > Off
            if (shooterActive) {
                shooterController.setTargetRpm(SHOOTER_RPM, nowMs);
            } else if (shooterIdle) {
                shooterController.setTargetRpm(SHOOTER_IDLE_RPM, nowMs);
            } else {
                shooterController.stop(nowMs);
            }
            
            // AUTO FIRE SANITY CHECK
            boolean isTracking = turretActiveMode;
            boolean hasTarget = (targetTag != null);
            boolean isAligned = Math.abs(targetTag != null ? targetTag.yaw : 999) < 4.0; 
            boolean isReady = shooterController.isReady(nowMs);
            
            boolean autoFeedCondition = isReady && isTracking && hasTarget && isAligned;

            // Failsafe: Manual Feed (G1 OR G2)
            boolean triggerPressed = (gamepad1.right_trigger > 0.5) || (gamepad2.right_trigger > 0.5);
            
            // Feed if Trigger is held OR Auto Condition is met
            shooterController.setFeedManual(triggerPressed || autoFeedCondition);
            
            // Boot only fires on MANUAL Trigger Rising Edge (G1 OR G2)
            // Boot Control: Manual Bidirectional (G1 or G2)
            // RT = Extend (Feed), LT or G2-DPAD-DOWN = Retract (Reverse/Unjam)
            boolean triggerReverse = (gamepad1.left_trigger > 0.5) || (gamepad2.left_trigger > 0.5) || gamepad2.dpad_down;
            shooterController.controlBoot(triggerPressed, triggerReverse, nowMs);
            triggerPrev = triggerPressed;
            
            shooterController.update(nowMs);

            // PRIORITY OVERRIDE: Outtake (X) reverses grips always
            // This must stay AFTER shooterController.update() to override its commands
            if (gamepad1.x) {
                if (hardware.grip1 != null) hardware.grip1.setPower(-1.0);
                if (hardware.grip2 != null) hardware.grip2.setPower(-1.0);
            }

            // SHOOTER REVERSE OVERRIDE (G2 Left Trigger)
            if (gamepad2.left_trigger > 0.5) {
                if (hardware.wheel != null) hardware.wheel.setPower(-1.0);
                if (hardware.wheel2 != null) hardware.wheel2.setPower(-1.0);
            }
            
            // --- Telemetry ---
            telemetry.addData("Alliance", getAllianceName());
            telemetry.addData("Shooter", shooterActive ? "ON" : "OFF");
            telemetry.addData("Turret Mode", turretActiveMode ? "ACTIVE" : "MANUAL");
            telemetry.addData("Tag Visible", hasTarget);
            telemetry.addData("Measured RPM", String.format("%.0f", shooterController.getMeasuredRpm()));
            
            // LED Status
            setLedStatus(isReady);
            
            telemetry.addData("Shooter Ready", isReady);
            telemetry.addData("Turret Lock", isAligned);
            telemetry.addData("Turret Deg", String.format("%.1f", currentDeg));
            telemetry.update();
        }
        
        camera.close();
    }
}
