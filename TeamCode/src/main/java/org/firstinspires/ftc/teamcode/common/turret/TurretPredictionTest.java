package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Test Program: Turret Prediction & Absolute Positioning
 * 
 * Objectives:
 * 1. Absolute Positioning: Use AprilTag to correct Turret Estimator belief.
 *    (Zeroing the encoder on the fly).
 * 2. Prediction/Compensation: Use IMU Gyro Rate to counter-rotate turret 
 *    when robot turns fast.
 * 3. Limits: 10 to 160 degrees.
 */
@TeleOp(name = "TEST: Turret Prediction", group = "Tests")
public class TurretPredictionTest extends LinearOpMode {

    private BjornHardware hardware;
    private AprilTagCamera camera;

    // Tuning
    public static double TURRET_KP = 0.035;
    public static double TURRET_KD = 0.002;
    public static double TURRET_KF_STATIC = 0.0; // Static friction overcome
    
    // Feedforward Gain for Robot Rotation (Counter-acting)
    // If robot rotates 1 deg/sec LEFT, Turret must rotate 1 deg/sec RIGHT.
    // Ideal gain is theoretically 1.0 (if motors were instant), usually close to 1.0 to 1.5.
    public static double ROBOT_ROTATION_FF_GAIN = 0.1; 

    // Limits
    public static double LOOK_MIN_DEG = 10.0;
    public static double LOOK_MAX_DEG = 160.0;

    @Override
    public void runOpMode() {
        hardware = BjornHardware.forTeleOp(hardwareMap);
        
        camera = new AprilTagCamera();
        camera.start(hardwareMap, null);

        // Assume Red Goal for test (ID 5)
        int targetId = CameraConfig.RED_GOAL_TAG_ID; 

        telemetry.addLine("Turret Prediction Test");
        telemetry.addLine("Press Start.");
        telemetry.update();

        waitForStart();

        double lastTime = getRuntime();
        double lastError = 0;
        double lastRobotYaw = 0;

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // 1. Hardware Update
            hardware.updateEstimator(); // x += encoderDelta

            // 2. Robot State
            YawPitchRollAngles angles = hardware.imu.getRobotYawPitchRollAngles();
            double robotYawDeg = angles.getYaw(AngleUnit.DEGREES);
            double robotYawRad = angles.getYaw(AngleUnit.RADIANS);
            
            // Calculate Robot Angular Velocity (deg/s) for Feedforward
            // Note: IMU might have getAngularVelocity(), simpler to derive here for control loop consistency
            double robotYawRate = (robotYawDeg - lastRobotYaw) / dt; 
            lastRobotYaw = robotYawDeg;

            // 3. Vision Update & Correction
            TagObservation target = null;
            List<TagObservation> detections = camera.pollDetections();
            for (TagObservation obs : detections) {
                if (obs.id == targetId) {
                    target = obs;
                    break;
                }
            }
            
            if (target != null) {
                // ABSOLUTE POSITIONING LOGIC:
                // We see the Tag. We know where the Tag Is (Target).
                // But simpler: Camera sees Tag at 'yaw' relative to Camera.
                // Turret Angle = (Angle To Tag Field Frame) - (Robot Heading) - (Tag Yaw).
                // Actually, if we just want "Where is the Turret Relative to Robot?":
                // If the Turret was PERFECTLY aiming at the tag, TagYaw = 0.
                // Current Belief says Turret is at 'x'.
                // The Observation implies the Turret is actually at 'x + TagYaw' ... wait.
                // If TagYaw is +10 (Left), it means the Tag is 10 deg Left of Center.
                // So the Turret is Pointing 10 deg RIGHT of the Tag.
                // Real Angle = ExpectedAngle - TagYaw?
                // 
                // Let's rely on "TagYaw" is the truth of error.
                // If we want to point at the tag, Error = -TagYaw.
                //
                // To correct the Estimator (which tracks 'Turret Angle Rel Robot'):
                // We need to know what the 'Turret Angle Rel Robot' SHOULD be if looking at tag.
                // TrueTurretAngle = FieldAngleTag - RobotYaw.
                // The measurement gives us: "We are at (TrueTurretAngle - TagYaw)".
                // Wait.
                // Let's use the Estimator for SMOOTHING, but correct it with Vision.
                // Observation z = (FieldAngleTag - RobotYaw) - SignalDelay?
                //
                // Simpler Approach for this Test:
                // "Absolute Degree Positioning" -> Use Tag to calibrate the Estimator.
                // Suppose Tag is at Field 0. Robot is at 90. Turret should be at -90.
                // Camera sees Tag at +5.
                // Means Turret is at -95.
                // So observedTurretAngle = (TagField - RobotYaw) - TagYaw.
                double tagFieldHeading = 0.0; // Assume Red Goal is "North" (0) or similar fixed point.
                // For simplicity, let's just assert the Tag is the origin.
                
                // Observed Turret Angle (Rel Robot) derived from Vision:
                // This assumes Tag 5 is at Field 0 degrees.
                double observedTurretRelRobot = tagFieldHeading - robotYawDeg - target.yaw;
                
                hardware.correctTurretBelief(observedTurretRelRobot);
            }
            
            // 4. Control Logic
            // Goal: Point at Field 0 (Tag).
            // TargetTurretRelRobot = TagFieldHeading - RobotYawDeg.
            double targetTurretHeading = 0.0 - robotYawDeg;
            
            // Normalize? No, turret has limits.
            
            double currentBelief = hardware.turretEstimator.getBelief();
            
            // Error
            double error = targetTurretHeading - currentBelief;
            
            // Derivative
            double dError = (error - lastError) / dt;
            lastError = error;

            // PID
            double pid = (error * TURRET_KP) + (dError * TURRET_KD);
            
            // Feedforward: Counter-rotate robot motion
            // If robot turns RIGHT (+rate), Turret must turn LEFT to stay fixed.
            // setPower(+) is Left? Check hardware. Usually (+) is Left/CCW.
            // If Robot Rate is + (Left), Turret needs - (Right).
            // FF = -1 * RobotRate * Gain.
            double ff = -robotYawRate * ROBOT_ROTATION_FF_GAIN; 

            // Static FF (Gravity/Friction) - simple sign
            double staticFF = Math.signum(error) * TURRET_KF_STATIC;

            double power = pid + ff + staticFF;

            // 5. Limits (10 to 160)
            if (currentBelief < LOOK_MIN_DEG && power < 0) power = 0;
            if (currentBelief > LOOK_MAX_DEG && power > 0) power = 0;

            // Apply
            hardware.turret.setPower(power);

            // 6. Drive (Simple Field Centric for Testing)
            double dy = -gamepad1.left_stick_y;
            double dx = gamepad1.left_stick_x * 1.1;
            double drx = gamepad1.right_stick_x;
            
            double rotX = dx * Math.cos(-robotYawRad) - dy * Math.sin(-robotYawRad);
            double rotY = dx * Math.sin(-robotYawRad) + dy * Math.cos(-robotYawRad);
            double den = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(drx), 1);
            
            hardware.frontLeft.setPower((rotY + rotX + drx) / den);
            hardware.frontRight.setPower((rotY - rotX - drx) / den);
            hardware.backLeft.setPower((rotY - rotX + drx) / den);
            hardware.backRight.setPower((rotY + rotX - drx) / den);

            // Telemetry
            telemetry.addData("Tag Visible", target != null);
            if (target != null) telemetry.addData("Tag Yaw", target.yaw);
            telemetry.addData("Robot Yaw", robotYawDeg);
            telemetry.addData("Robot Rate", robotYawRate);
            telemetry.addData("Turret Belief", currentBelief);
            telemetry.addData("Target Heading", targetTurretHeading);
            telemetry.addData("Error", error);
            telemetry.addData("Power", power);
            telemetry.addData("FF", ff);
            telemetry.update();
        }
    }
}
