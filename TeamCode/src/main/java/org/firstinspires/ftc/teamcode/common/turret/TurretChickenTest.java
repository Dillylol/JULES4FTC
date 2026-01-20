package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Turret "Chicken Head" Test
 * 
 * Demonstrates:
 * 1. IMU-based Field Locking ("Chicken Head" mode): Turret stays pointing at a Field Angle
 *    regardless of robot rotation.
 * 2. Active Vision Lock: Turret tracks AprilTag.
 * 3. Feedforward Counter-Rotation: Uses Gyro Rate to preemptively turn turret against robot turn.
 * 
 * Controls:
 * - A: Toggle "Chicken Lock" (Locks to CURRENT Field Angle)
 * - B: Toggle "Active Tag Lock" (Locks to Tag ID 5/Red Goal)
 * - Y: Reset Field Angle Target to 0 (Forward)
 * - LS: Drive
 * - RS: Turn Robot
 */
@TeleOp(name = "TEST: Turret Chicken Head", group = "Tests")
public class TurretChickenTest extends LinearOpMode {

    private BjornHardware hardware;
    private AprilTagCamera camera;

    // --- Tuning Constants ---
    public static double TURRET_KP = 0.03;  // Proportional Gain
    public static double TURRET_KD = 0.002; // Derivative Gain
    
    // Feedforward Gain: How much we counter-act Robot Rotation.
    // 1.0 = Theoretical perfect counter-action (if motors were instant and 1:1 mapped).
    // Start low and tune up.
    public static double ROBOT_ROTATION_FF_GAIN = 0.15; 

    // Limits
    public static double LIMIT_MIN = 10.0;
    public static double LIMIT_MAX = 160.0;

    // State
    private boolean chickenLockActive = false;
    private boolean tagLockActive = false;
    private double targetFieldHeading = 0.0; // The Field Angle we want to look at
    
    private boolean aPrev = false;
    private boolean bPrev = false;
    private boolean yPrev = false;

    @Override
    public void runOpMode() {
        hardware = BjornHardware.forTeleOp(hardwareMap);
        camera = new AprilTagCamera();
        camera.start(hardwareMap, null);

        telemetry.addLine("Turret Chicken Test");
        telemetry.addLine("A: Toggle Chicken Lock (Holds Current Angle)");
        telemetry.addLine("B: Toggle Tag Lock (Tracks Red Goal)");
        telemetry.addLine("Y: Reset Target to 0 (Forward)");
        telemetry.update();

        waitForStart();

        // Loop Variables
        double lastTime = getRuntime();
        double lastError = 0;
        double lastRobotYaw = 0;

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // 1. Sensors
            hardware.updateEstimator(); // Update Encoder Belief
            YawPitchRollAngles angles = hardware.imu.getRobotYawPitchRollAngles();
            double robotYaw = angles.getYaw(AngleUnit.DEGREES);
            double robotYawRad = angles.getYaw(AngleUnit.RADIANS);
            
            // Calculate Robot Rotation Rate (deg/s) for Feedforward
            double robotRate = (robotYaw - lastRobotYaw) / dt;
            lastRobotYaw = robotYaw;

            // Vision
            TagObservation target = null;
            List<TagObservation> dets = camera.pollDetections();
            for (TagObservation obs : dets) {
                if (obs.id == CameraConfig.RED_GOAL_TAG_ID) {
                    target = obs;
                    break;
                }
            }

            // 2. Input
            if (gamepad1.a && !aPrev) {
                chickenLockActive = !chickenLockActive;
                tagLockActive = false; // Priority logic
                if (chickenLockActive) {
                    // Lock to CURRENT look direction
                    // Current Field Angle = RobotYaw + TurretAngle
                    targetFieldHeading = robotYaw + hardware.turretEstimator.getBelief();
                }
            }
            aPrev = gamepad1.a;

            if (gamepad1.b && !bPrev) {
                tagLockActive = !tagLockActive;
                chickenLockActive = false;
            }
            bPrev = gamepad1.b;

            if (gamepad1.y && !yPrev) {
                targetFieldHeading = 0.0; // Reset to Forward
                chickenLockActive = true;
                tagLockActive = false;
            }
            yPrev = gamepad1.y;

            // 3. Logic
            double turretPower = 0.0;
            double currentBelief = hardware.turretEstimator.getBelief();

            if (tagLockActive) {
                if (target != null) {
                    // Active Tracking Loop
                    // Error is simply -TagYaw (we want TagYaw to be 0)
                    // If TagYaw is +10 (Left), we must turn Left (+).
                    // WAIT: Camera/Turret Setup.
                    // If Tag is 10deg Left, we need to turn Turret +10deg.
                    // So Error = TagYaw?
                    // Previous logic used Error = -TagYaw. Let's verify sign.
                    // If Tag is Left (+), we want Turret to go Left (+).
                    // Power = KP * Error. So Error should be +.
                    // So Error = TagYaw.
                    double error = target.yaw; 
                    
                    // Update TargetFieldHeading for fallback/hybrid?
                    // TargetField = Robot + Turret + Yaw.
                    targetFieldHeading = robotYaw + currentBelief + target.yaw;

                    // PID
                    double derivative = (error - lastError) / dt;
                    double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
                    
                    // Feedforward
                    // If Robot Rate is + (Left), Turret needs - (Right) to stay locally fixed,
                    // BUT here we are tracking a moving/stationary target relative to robot.
                    // The feedforward is still valid: Counter-act robot motion.
                    double ff = -robotRate * ROBOT_ROTATION_FF_GAIN;
                    
                    turretPower = pid + ff;
                    lastError = error;
                    
                } else {
                    // Tag Lost -> Fallback to Chicken Lock on last known heading?
                    // Or stop. Let's stop to be safe, or hold position.
                    turretPower = 0.0;
                }
            } else if (chickenLockActive) {
                // CHICKEN HEAD MODE
                // Goal: Turret Field Angle == Target Field Heading
                // Turret Field Angle = RobotYaw + TurretRelAngle
                // So: RobotYaw + TurretRelAngle = TargetFieldHeading
                // TurretRelAngle (Desired) = TargetFieldHeading - RobotYaw
                
                double desiredRel = targetFieldHeading - robotYaw;
                
                // Wrap angle if needed? (Turret has limits, so probably not full 360 wrap)
                
                double error = desiredRel - currentBelief;
                
                double derivative = (error - lastError) / dt;
                double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
                
                double ff = -robotRate * ROBOT_ROTATION_FF_GAIN;
                
                turretPower = pid + ff;
                lastError = error;
            } else {
                // Manual / Idle
                turretPower = gamepad1.left_trigger - gamepad1.right_trigger; // Manual test
            }

            // 4. Limits
            if (currentBelief < LIMIT_MIN && turretPower < 0) turretPower = 0;
            if (currentBelief > LIMIT_MAX && turretPower > 0) turretPower = 0;

            // 5. Drive (Field Centric with Bjorn Speeds)
            // Speeds from BaseBjornTeleOp
            double DRIVE_SPEED = 1.0;
            double TURN_SPEED = 0.7;

            // IMU Reset
            if (gamepad1.dpad_down) {
                hardware.imu.resetYaw();
            }

            double dy = -gamepad1.left_stick_y * DRIVE_SPEED;
            double dx = gamepad1.left_stick_x * DRIVE_SPEED * 1.1; // Counteract imperfect strafing
            double drx = gamepad1.right_stick_x * TURN_SPEED;
            
            double rotX = dx * Math.cos(-robotYawRad) - dy * Math.sin(-robotYawRad);
            double rotY = dx * Math.sin(-robotYawRad) + dy * Math.cos(-robotYawRad);
            double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(drx), 1);
            
            hardware.frontLeft.setPower((rotY + rotX + drx) / den);
            hardware.frontRight.setPower((rotY - rotX - drx) / den);
            hardware.backLeft.setPower((rotY - rotX + drx) / den);
            hardware.backRight.setPower((rotY + rotX - drx) / den);

            // Cap Turret Power to 0.5 (Absolute Lowest)
            if (Double.isNaN(turretPower)) turretPower = 0;
            turretPower = Range.clip(turretPower, -0.5, 0.5); // CAPPED at 0.5
            
            hardware.turret.setPower(turretPower);

            // Telemetry
            telemetry.addData("Mode", tagLockActive ? "TAG" : (chickenLockActive ? "CHICKEN" : "MANUAL"));
            telemetry.addData("Target Heading", "%.1f", targetFieldHeading);
            telemetry.addData("Robot Yaw", "%.1f", robotYaw);
            telemetry.addData("Turret Angle", "%.1f", currentBelief);
            telemetry.addData("Robot Rate", "%.1f", robotRate);
            telemetry.addData("Power", "%.2f", turretPower);
            if (target != null) telemetry.addData("Tag Yaw", "%.1f", target.yaw);
            telemetry.update();
        }
    }
}
