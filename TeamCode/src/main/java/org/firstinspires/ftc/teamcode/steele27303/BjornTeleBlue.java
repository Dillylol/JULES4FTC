package org.firstinspires.ftc.teamcode.steele27303;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

/**
 * BLUE TeleOp - Drive/IMU/Pedro/Camera/Turret Inline (Reduced Latency)
 * 
 * Gamepad 1 (Driver):
 * - Left Stick: Drive/Strafe
 * - Right Stick: Rotate
 * - D-Pad Down: IMU Reset
 * - D-Pad L/R: Manual turret at 0.5 speed (limited authority)
 * - A: Intake
 * - X: Outtake
 * - LB: Mark waypoint
 * - RB: Execute auto-drive
 * 
 * Gamepad 2 (Gunner):
 * - Left Stick X: Turret rotation (0.75 max power)
 * - A: Toggle AprilTag tracking
 * - B: Toggle shooter ramp-up
 * - RT: Fire (boot kick)
 */
@TeleOp(name = "Bjorn TeleOp BLUE", group = "TeleOp")
public class BjornTeleBlue extends BjornTeleBase {

    // --- Drive Hardware (Inline) ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // --- Turret Hardware (Inline) ---
    private DcMotorEx turret;
    private DistanceSensor frontTof;

    // --- Pedro (Inline) ---
    private Follower follower;
    private Pose autoDriveTargetPose;
    private boolean autoDriveActive = false;

    // --- Camera (Inline) ---
    private AprilTagCamera aprilTagCamera;
    private static final int GOAL_TAG_ID = CameraConfig.BLUE_GOAL_TAG_ID;

    // --- Turret Constants ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 5.0 * 4.0 * 4.0; // 80:1
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;
    
    private static final double TURRET_KP = 0.03;
    private static final double TURRET_KD = 0.0008;
    private static final double ROBOT_ROTATION_FF_GAIN = 0.0025;
    private static final double DEADBAND_DEG = 2.0;
    private static final double MAX_POWER_DELTA = 0.04;
    private static final double LIMIT_MIN = 20.0;
    private static final double LIMIT_MAX = 160.0;
    private static final double MANUAL_RATE_DEG_PER_SEC = 90.0;
    private static final double APRILTAG_CORRECTION_GAIN = 0.5;
    
    private static final double G2_TURRET_POWER = 0.75;
    private static final double G1_TURRET_POWER = 0.5;

    // --- Turret State ---
    private double targetFieldHeading = 90.0;
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastRobotYaw = 0.0;
    private boolean aprilTagTrackingEnabled = false;
    private boolean wasManualControl = false;

    // --- Input State ---
    private boolean g1LbPrev = false;
    private boolean yawResetPrev = false;
    private boolean g2APrev = false;
    private double lastLoopTime = 0;

    // --- Alliance Config ---
    private static final Pose AUTO_END_POSE = BjornConstants.Auto.BLUE_AUTO_END_POSE;

    @Override
    protected String getAllianceName() {
        return "BLUE";
    }
    
    @Override
    protected int getGoalTagId() {
        return GOAL_TAG_ID;
    }

    @Override
    public void init() {
        // --- Drive Motors (Inline Init) ---
        frontLeft = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.FRONT_LEFT);
        frontRight = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.FRONT_RIGHT);
        backLeft = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.BACK_LEFT);
        backRight = hardwareMap.get(DcMotorEx.class, BjornConstants.Motors.BACK_RIGHT);

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        // --- Turret Motor (Inline Init) ---
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU (Inline Init) ---
        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        // --- Pedro Follower (Inline Init) ---
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(AUTO_END_POSE);
        autoDriveTargetPose = AUTO_END_POSE;

        // --- AprilTag Camera (Inline Init) ---
        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);

        // --- ToF Sensor (Inline Init) ---
        frontTof = hardwareMap.get(DistanceSensor.class, BjornConstants.Sensors.TOF_FRONT);

        // --- Subsystems (From Base) ---
        initSubsystems();

        telemetry.addLine("Bjorn TeleOp BLUE Initialized");
        telemetry.addData("Goal Tag", GOAL_TAG_ID);
        telemetry.update();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // --- Update Systems (Inline) ---
        follower.update();

        // --- Input Handling ---
        handleWaypointInput();
        handleImuReset();

        // --- Turret Control (Inline - Latency Sensitive) ---
        updateTurret(dt);

        // --- Drive (Inline) ---
        updateDrive();

        // --- Subsystems (From Base) - G2 controls shooter/boot ---
        // Override shooter RPM if active
        if (shooterActive) {
            shooterController.setTargetRpm(getDynamicRpm(), nowMs);
        }
        updateSubsystems(nowMs);

        // --- Telemetry ---
        telemetry.addData("Alliance", "BLUE");
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
        telemetry.addData("Turret", "%.1f° | Target: %.1f°", turretAngleDeg, targetFieldHeading);
        telemetry.addData("Tracking", aprilTagTrackingEnabled ? "ON" : "OFF");
        telemetry.addData("Auto-Drive", autoDriveActive ? "ACTIVE" : "OFF");
        addSubsystemTelemetry(nowMs);
        telemetry.update();
    }

    // --- Turret Control (Inline - From InclusiveTurretTest) ---

    private void updateTurret(double dt) {
        // Update turret angle from encoder
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        turretAngleDeg += deltaTicks / TURRET_TICKS_PER_DEGREE;

        // Get robot rotation
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
        lastRobotYaw = robotYaw;

        // --- G2: A toggles AprilTag tracking ---
        if (gamepad2.a && !g2APrev) {
            aprilTagTrackingEnabled = !aprilTagTrackingEnabled;
        }
        g2APrev = gamepad2.a;

        // --- Manual Control ---
        // G2: Left stick X (full authority at 0.75)
        // G1: D-Pad L/R (limited authority at 0.5)
        double g2Input = -gamepad2.left_stick_x;
        double g1Input = 0.0;
        if (gamepad1.dpad_left) g1Input = -1.0;
        else if (gamepad1.dpad_right) g1Input = 1.0;

        // Combine inputs (G2 has priority if both active)
        double stickInput = (Math.abs(g2Input) > 0.1) ? g2Input : g1Input * (G1_TURRET_POWER / G2_TURRET_POWER);
        boolean isManualControl = Math.abs(stickInput) > 0.1;

        if (isManualControl) {
            double headingDelta = stickInput * MANUAL_RATE_DEG_PER_SEC * dt;
            targetFieldHeading += headingDelta;

            // Clamp to limits
            double minFieldHeading = robotYaw + LIMIT_MIN;
            double maxFieldHeading = robotYaw + LIMIT_MAX;
            targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);
            wasManualControl = true;
        } else if (wasManualControl) {
            // Just released - lock to current position
            targetFieldHeading = robotYaw + turretAngleDeg;
            wasManualControl = false;
        }

        // --- AprilTag Tracking (Non-blocking) ---
        TagObservation target = null;
        boolean tagVisible = false;

        if (aprilTagTrackingEnabled) {
            List<TagObservation> detections = aprilTagCamera.pollDetections();
            for (TagObservation obs : detections) {
                if (obs.id == GOAL_TAG_ID) {
                    target = obs;
                    tagVisible = true;
                    break;
                }
            }

            if (tagVisible && !isManualControl) {
                targetFieldHeading += target.yaw * APRILTAG_CORRECTION_GAIN * dt * 30.0;
                double minFieldHeading = robotYaw + LIMIT_MIN;
                double maxFieldHeading = robotYaw + LIMIT_MAX;
                targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);
            }
        } else {
            // Still poll for telemetry even if not tracking
            aprilTagCamera.pollDetections();
        }

        // --- Heading Hold PD Control ---
        double desiredTurretAngle = targetFieldHeading - robotYaw;
        double error = desiredTurretAngle - turretAngleDeg;

        if (Math.abs(error) < DEADBAND_DEG) {
            error = 0;
        }

        double derivative = (dt > 0) ? (error - lastTurretError) / dt : 0.0;
        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
        double ff = -robotRate * ROBOT_ROTATION_FF_GAIN;

        double turretPower = pid + ff;
        lastTurretError = error;

        // Apply limits
        if (turretAngleDeg < LIMIT_MIN && turretPower < 0) turretPower = 0;
        if (turretAngleDeg > LIMIT_MAX && turretPower > 0) turretPower = 0;

        // Clamp and slew rate limit
        if (Double.isNaN(turretPower)) turretPower = 0;
        turretPower = Range.clip(turretPower, -G2_TURRET_POWER, G2_TURRET_POWER);

        double powerDelta = Range.clip(turretPower - lastTurretPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
        turretPower = lastTurretPower + powerDelta;
        lastTurretPower = turretPower;

        turret.setPower(turretPower);
    }

    // --- Inline Input Handlers ---

    private void handleWaypointInput() {
        boolean markWaypoint = gamepad1.left_bumper;
        if (markWaypoint && !g1LbPrev) {
            autoDriveTargetPose = follower.getPose();
        }
        g1LbPrev = markWaypoint;
    }

    private void handleImuReset() {
        boolean yawReset = gamepad1.dpad_down;
        if (yawReset && !yawResetPrev) {
            imu.resetYaw();
            follower.setStartingPose(new Pose(
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    0));
            // Reset turret target to match
            targetFieldHeading = turretAngleDeg;
        }
        yawResetPrev = yawReset;
    }

    // --- Inline Drive ---

    private void updateDrive() {
        if (gamepad1.right_bumper) {
            if (!autoDriveActive) {
                Path path = new Path(new BezierLine(follower.getPose(), autoDriveTargetPose));
                path.setLinearHeadingInterpolation(follower.getPose().getHeading(), autoDriveTargetPose.getHeading());
                follower.followPath(path, true);
                autoDriveActive = true;
            }
            return;
        } else {
            if (autoDriveActive) {
                follower.startTeleopDrive();
                autoDriveActive = false;
            }
        }

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        frontLeft.setPower((rotY + rotX + rx) / denom);
        backLeft.setPower((rotY - rotX + rx) / denom);
        frontRight.setPower((rotY - rotX - rx) / denom);
        backRight.setPower((rotY + rotX - rx) / denom);
    }

    private double getDynamicRpm() {
        double rangeFt = -1.0;
        boolean usingCv = false;

        // 1. Try CV (Primary)
        TagObservation goal = aprilTagCamera.getLatestGoalObservation(); // Assuming method exists or we poll
        // Need to check latest observations since getLatestGoalObservation might return stale or null from thread 
        // Iterate detections again cleanly or use helper
        List<TagObservation> detections = aprilTagCamera.pollDetections();
        for (TagObservation obs : detections) {
            if (obs.id == GOAL_TAG_ID) {
                double x = obs.x / 0.0254; // meters to inches
                double y = obs.y / 0.0254;
                double z = obs.z / 0.0254; 
                double distIn = Math.sqrt(x*x + y*y + z*z);
                rangeFt = distIn / 12.0;
                usingCv = true;
                break;
            }
        }

        double rpm;
        double slope, offset;
        
        if (usingCv) {
             slope = BjornConstants.Power.SHOOTER_RPM_SLOPE_CV;
             offset = BjornConstants.Power.SHOOTER_RPM_OFFSET_CV;
        } else {
             // 2. Fallback to ToF
             double distIn = frontTof.getDistance(DistanceUnit.INCH);
             if (distIn > 100 || Double.isNaN(distIn)) { // Filter bad data
                 distIn = 0; // Default safe value? Or keep last valid? 
                 // If invalid, let's just clamp to min meaningful range or set a reasonable default
                 rangeFt = 4.0; // Assume close range shot if sensor fails?
             } else {
                 rangeFt = distIn / 12.0;
             }
             slope = BjornConstants.Power.SHOOTER_RPM_SLOPE_TOF;
             offset = BjornConstants.Power.SHOOTER_RPM_OFFSET_TOF;
        }

        // 3. Calc & Clamp
        rpm = (slope * rangeFt) + offset;
        rpm = Range.clip(rpm, BjornConstants.Power.SHOOTER_MIN_RPM, BjornConstants.Power.SHOOTER_MAX_RPM);
        
        telemetry.addData("DynRPM", "%.0f (Range: %.1fft, Source: %s)", rpm, rangeFt, usingCv ? "CV" : "ToF");
        
        return rpm;
    }
}
