package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Hybrid Fusion Tracking Test - IMU + CV Fusion (STANDALONE RIG VERSION)
 * 
 * For testing on a rig with only: turret motor, IMU, camera, controller
 * No drive motors required.
 * 
 * State Machine:
 * - IDLE: No tracking, manual control available
 * - TRACKING: Following AprilTag using camera yaw
 * - LOCKED: Tag acquired, holding heading via IMU
 * - LOST: Heading lost, turret stops (safety exit)
 * 
 * Flow:
 * IDLE -> TRACKING (press A)
 * TRACKING -> LOCKED (tag centered for 500ms)
 * LOCKED -> LOST (tag lost for 1s OR limits hit)
 * LOST -> TRACKING (tag regained)
 * Any -> IDLE (press B)
 * 
 * Controls:
 * - A: Start tracking
 * - B: Cancel/Stop (return to IDLE)
 * - D-Pad Down: Reset IMU
 * - Y: Reset turret angle to 0
 * 
 * Limits: 20° to 160°
 */
@TeleOp(name = "TEST: Hybrid Fusion Tracking (Rig)", group = "Tests")
public class HybridFusionTrackingTest extends LinearOpMode {

    // Hardware - defined directly for standalone rig testing
    private DcMotorEx turret;
    private IMU imu;
    private AprilTagCamera camera;

    // Gear ratio constants
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 5.0 * 4.0 * 4.0; // 80:1
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    // --- States ---
    private enum TrackingState {
        IDLE,
        TRACKING,
        LOCKED,
        LOST
    }

    // --- Tuning Constants (Matched from InertialTurretTest) ---
    public static double TURRET_KP = 0.03;
    public static double TURRET_KD = 0.0008;
    public static double ROBOT_ROTATION_FF_GAIN = 0.0025;

    // Deadband: Ignore small errors to reduce jitter
    public static double DEADBAND_DEG = 2.0;

    // Slew Rate: Max power change per loop to smooth output
    public static double MAX_POWER_DELTA = 0.04;

    // State transition parameters
    private static final double LOCK_THRESHOLD_DEG = 3.0;
    private static final long LOCK_SETTLE_MS = 500;
    private static final long LOST_TIMEOUT_MS = 1000;

    // Limits
    public static double LIMIT_MIN = 20.0;
    public static double LIMIT_MAX = 160.0;

    // State
    private TrackingState state = TrackingState.IDLE;
    private double lockedFieldHeading = 0.0;
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private long centeredSinceMs = 0;
    private long tagLastSeenMs = 0;
    private double lastPower = 0.0;

    private boolean aPrev = false;
    private boolean bPrev = false;

    @Override
    public void runOpMode() {
        // Initialize turret motor
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        // Initialize camera
        camera = new AprilTagCamera();
        camera.start(hardwareMap, null);

        telemetry.addLine("Hybrid Fusion Tracking Test (Rig)");
        telemetry.addLine("A: Start tracking");
        telemetry.addLine("B: Cancel/Stop");
        telemetry.addLine("D-Pad Down: Reset IMU");
        telemetry.addLine("Y: Reset turret angle");
        telemetry.update();

        waitForStart();

        double lastTime = getRuntime();
        double lastError = 0;
        double lastRobotYaw = 0;
        lastEncoderPos = turret.getCurrentPosition();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // Update turret angle from encoder
            int currentPos = turret.getCurrentPosition();
            int deltaTicks = currentPos - lastEncoderPos;
            lastEncoderPos = currentPos;
            turretAngleDeg += deltaTicks / TURRET_TICKS_PER_DEGREE;

            // Get robot/rig rotation
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double robotYaw = angles.getYaw(AngleUnit.DEGREES);

            double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
            lastRobotYaw = robotYaw;

            // Vision - Find any goal tag (blue or red)
            TagObservation target = null;
            List<TagObservation> detections = camera.pollDetections();
            for (TagObservation obs : detections) {
                if (CameraConfig.isGoalTag(obs.id)) {
                    target = obs;
                    break;
                }
            }

            boolean tagVisible = (target != null);
            if (tagVisible) {
                tagLastSeenMs = nowMs;
            }

            // Input
            if (gamepad1.a && !aPrev) {
                if (state == TrackingState.IDLE || state == TrackingState.LOST) {
                    state = TrackingState.TRACKING;
                    centeredSinceMs = 0;
                }
            }
            aPrev = gamepad1.a;

            if (gamepad1.b && !bPrev) {
                state = TrackingState.IDLE;
                centeredSinceMs = 0;
            }
            bPrev = gamepad1.b;

            if (gamepad1.dpad_down) {
                imu.resetYaw();
            }

            // Y to reset turret angle
            if (gamepad1.y) {
                turretAngleDeg = 0.0;
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastEncoderPos = 0;
            }

            // State Machine Logic
            double turretPower = 0.0;
            double error = 0.0;

            switch (state) {
                case IDLE:
                    // Manual control with triggers
                    turretPower = (gamepad1.left_trigger - gamepad1.right_trigger) * 0.4;
                    break;

                case TRACKING:
                    if (tagVisible) {
                        // Track the tag using camera yaw
                        error = target.yaw;

                        // Apply deadband
                        if (Math.abs(error) < DEADBAND_DEG) {
                            error = 0;
                        }

                        // Update locked heading continuously
                        lockedFieldHeading = robotYaw + turretAngleDeg;

                        // PD Controller
                        double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
                        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
                        double ff = -robotRate * ROBOT_ROTATION_FF_GAIN;

                        turretPower = pid + ff;
                        lastError = error;

                        // Check for lock transition
                        if (Math.abs(target.yaw) < LOCK_THRESHOLD_DEG) {
                            if (centeredSinceMs == 0) {
                                centeredSinceMs = nowMs;
                            } else if (nowMs - centeredSinceMs >= LOCK_SETTLE_MS) {
                                state = TrackingState.LOCKED;
                                lockedFieldHeading = robotYaw + turretAngleDeg;
                            }
                        } else {
                            centeredSinceMs = 0;
                        }
                    } else {
                        turretPower = 0.0;
                        centeredSinceMs = 0;
                    }
                    break;

                case LOCKED:
                    // Hold heading via IMU
                    double desiredRel = lockedFieldHeading - robotYaw;
                    error = desiredRel - turretAngleDeg;

                    // Apply deadband
                    if (Math.abs(error) < DEADBAND_DEG) {
                        error = 0;
                    }

                    double derivLocked = (dt > 0) ? (error - lastError) / dt : 0.0;
                    double pidLocked = (error * TURRET_KP) + (derivLocked * TURRET_KD);
                    double ffLocked = -robotRate * ROBOT_ROTATION_FF_GAIN;

                    turretPower = pidLocked + ffLocked;
                    lastError = error;

                    // If tag visible, slight correction
                    if (tagVisible && Math.abs(target.yaw) < LOCK_THRESHOLD_DEG * 2) {
                        lockedFieldHeading = robotYaw + turretAngleDeg + (target.yaw * 0.1);
                    }

                    // Check for LOST transition
                    boolean atLimit = (turretAngleDeg <= LIMIT_MIN && turretPower < 0) ||
                            (turretAngleDeg >= LIMIT_MAX && turretPower > 0);
                    boolean tagLostTimeout = (nowMs - tagLastSeenMs) > LOST_TIMEOUT_MS;

                    if (atLimit || tagLostTimeout) {
                        state = TrackingState.LOST;
                    }
                    break;

                case LOST:
                    // Safety exit - stop turret movement
                    turretPower = 0.0;

                    // Check for recovery
                    if (tagVisible) {
                        state = TrackingState.TRACKING;
                        centeredSinceMs = 0;
                    }
                    break;
            }

            // Apply Limits (only in non-LOST states)
            if (state != TrackingState.LOST) {
                if (turretAngleDeg < LIMIT_MIN && turretPower < 0)
                    turretPower = 0;
                if (turretAngleDeg > LIMIT_MAX && turretPower > 0)
                    turretPower = 0;
            }

            // Cap turret power
            if (Double.isNaN(turretPower))
                turretPower = 0;
            turretPower = Range.clip(turretPower, -0.75, 0.75);

            // Slew rate limiting
            double powerDelta = Range.clip(turretPower - lastPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
            turretPower = lastPower + powerDelta;
            lastPower = turretPower;

            turret.setPower(turretPower);

            // Telemetry
            telemetry.addData("STATE", state.name());
            telemetry.addLine();
            telemetry.addData("Tag Visible", tagVisible);
            if (tagVisible) {
                telemetry.addData("Tag ID", target.id);
                telemetry.addData("Tag Yaw", "%.1f°", target.yaw);
            }
            telemetry.addLine();
            telemetry.addData("Robot/Rig Yaw", "%.1f°", robotYaw);
            telemetry.addData("Turret Angle", "%.1f°", turretAngleDeg);
            telemetry.addData("Locked Heading", "%.1f°", lockedFieldHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Turret Power", "%.2f", turretPower);
            telemetry.addLine();
            telemetry.addData("Limits", "%.0f° - %.0f°", LIMIT_MIN, LIMIT_MAX);

            if (state == TrackingState.TRACKING && centeredSinceMs > 0) {
                telemetry.addData("Lock Progress", "%d/%d ms",
                        (int) (nowMs - centeredSinceMs), (int) LOCK_SETTLE_MS);
            }

            telemetry.update();
        }

        // Cleanup
        if (camera != null) {
            camera.close();
        }
    }
}
