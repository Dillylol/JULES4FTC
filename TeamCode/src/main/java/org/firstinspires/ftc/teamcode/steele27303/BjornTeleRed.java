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
 * RED TeleOp - Drive/IMU/Pedro/Camera/Turret Inline (Reduced Latency)
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
@TeleOp(name = "Bjorn TeleOp RED", group = "TeleOp")
public class BjornTeleRed extends BjornTeleBase {

    // --- Drive Hardware (Inline) ---
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // --- Turret Hardware (Inline) ---
    private DcMotorEx turret;
    // frontTof moved to Base/Hardware

    // --- Pedro (Inline) ---
    private Follower follower;
    private Pose autoDriveTargetPose;
    private boolean autoDriveActive = false;

    // --- Camera (Inline) ---
    private AprilTagCamera aprilTagCamera;
    private static final int GOAL_TAG_ID = CameraConfig.RED_GOAL_TAG_ID;

    // --- Turret Constants ---
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 75.52; // Actual ratio (5.23 * 3.61 * 4)
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    private static final double TURRET_KP = 0.015;
    private static final double TURRET_KD = 0.004;
    private static final double ROBOT_ROTATION_FF_GAIN = 0.0055;
    private static final double DEADBAND_DEG = 2.0;
    private static final double MAX_POWER_DELTA = 0.15;
    private static final double LIMIT_MIN = 30.0;
    private static final double LIMIT_MAX = 155.0;
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
    private boolean dpadUpPrev = false; // Added for G1 Camera Toggle
    private double lastLoopTime = 0;

    // --- Alliance Config ---
    private static final Pose AUTO_END_POSE = BjornConstants.Auto.RED_AUTO_END_POSE;

    @Override
    protected String getAllianceName() {
        return "RED";
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
        // frontTof initialized in Base via BjornHardware

        // --- Subsystems (From Base) ---
        initSubsystems();

        // Pass Camera to Shooter for CV calculation
        shooter.setCamera(aprilTagCamera, GOAL_TAG_ID);

        telemetry.addLine("Bjorn TeleOp RED Initialized");
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
        // Shooter RPM managed by TeleOpShooter in updateSubsystems
        updateSubsystems(nowMs);

        // --- Telemetry ---
        telemetry.addData("Alliance", "RED");
        telemetry.addData("Heading", "%.1f°",
                Math.toDegrees(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)));
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
        turretAngleDeg += (deltaTicks / TURRET_TICKS_PER_DEGREE) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;

        // Get robot rotation
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
        lastRobotYaw = robotYaw;

        // --- G2: A toggles AprilTag tracking ---
        // Added: G1 D-Pad Up also toggles tracking
        if ((gamepad2.a && !g2APrev) || (gamepad1.dpad_up && !dpadUpPrev)) {
            aprilTagTrackingEnabled = !aprilTagTrackingEnabled;
        }
        g2APrev = gamepad2.a;
        dpadUpPrev = gamepad1.dpad_up;

        // --- Manual Control ---
        // G2: Left stick X (full authority at 0.75)
        // G1: D-Pad L/R (limited authority at 0.5)
        double g2Input = gamepad2.left_stick_x;
        double g1Input = 0.0;
        if (gamepad1.dpad_left)
            g1Input = -1.0;
        else if (gamepad1.dpad_right)
            g1Input = 1.0;

        // Combine inputs (G2 has priority if both active)
        double stickInput = (Math.abs(g2Input) > 0.1) ? g2Input : g1Input * (G1_TURRET_POWER / G2_TURRET_POWER);
        boolean isManualControl = Math.abs(stickInput) > 0.1;

        if (isManualControl) {
            double headingDelta = stickInput * MANUAL_RATE_DEG_PER_SEC * dt;
            targetFieldHeading += headingDelta;

            // Clamp to limits (Target = Local - Yaw)
            double minFieldHeading = LIMIT_MIN - robotYaw;
            double maxFieldHeading = LIMIT_MAX - robotYaw;
            targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);
            wasManualControl = true;
        } else if (wasManualControl) {
            // Just released - lock to current position (Target = Local - Yaw)
            targetFieldHeading = turretAngleDeg - robotYaw;
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
                // Filter Hallucinations: Check range (max 6.0 meters / ~20 ft)
                double dist = Math.hypot(target.x, target.y); // Horizontal dist
                if (dist < 6.0) {
                    targetFieldHeading += target.yaw * APRILTAG_CORRECTION_GAIN * dt * 30.0
                            * CameraConfig.CAMERA_TO_TURRET_SCALAR;
                    double minFieldHeading = LIMIT_MIN - robotYaw;
                    double maxFieldHeading = LIMIT_MAX - robotYaw;
                    targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);
                }
            }
        } else {
            // Still poll for telemetry even if not tracking
            aprilTagCamera.pollDetections();
        }

        // --- Heading Hold PD Control ---
        // --- Heading Hold PD Control ---
        double desiredTurretAngle = targetFieldHeading + robotYaw;

        // ABSOLUTE SAFETY CLAMP: Ensure target never exceeds physical limits
        desiredTurretAngle = Range.clip(desiredTurretAngle, LIMIT_MIN, LIMIT_MAX);

        double error = desiredTurretAngle - turretAngleDeg;

        if (Math.abs(error) < DEADBAND_DEG) {
            error = 0;
        }

        double derivative = (dt > 0) ? (error - lastTurretError) / dt : 0.0;
        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
        double ff = robotRate * ROBOT_ROTATION_FF_GAIN;

        double turretPower = pid + ff;
        lastTurretError = error;

        // Apply limits
        // Apply limits (Standard PID suppression)
        if (turretAngleDeg < LIMIT_MIN && turretPower < 0)
            turretPower = 0;
        if (turretAngleDeg > LIMIT_MAX && turretPower > 0)
            turretPower = 0;

        // Clamp and slew rate limit
        // Clamp and slew rate limit (Standard)
        if (Double.isNaN(turretPower))
            turretPower = 0;
        turretPower = Range.clip(turretPower, -G2_TURRET_POWER, G2_TURRET_POWER);

        double powerDelta = Range.clip(turretPower - lastTurretPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
        double slewPower = lastTurretPower + powerDelta;

        // --- FORCE FIELD OVERRIDE (Bypasses Slew Rate) ---
        // If we are out of bounds, we ignore slew rate and apply IMMEDIATE corrective
        // force.
        if (turretAngleDeg < LIMIT_MIN) {
            slewPower = 0.6; // Immediate Hard Push Right
        } else if (turretAngleDeg > LIMIT_MAX) {
            slewPower = -0.6; // Immediate Hard Push Left
        }

        turretPower = slewPower;
        lastTurretPower = turretPower;

        turret.setPower(turretPower * BjornConstants.Motors.TURRET_POWER_DIRECTION);
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
            // Reset turret target to match (Target = Local - Yaw)
            targetFieldHeading = turretAngleDeg - 0.0;
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

    // getDynamicRpm removed - logic moved to TeleOpShooter
}
