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
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * Inclusive Turret Test - Field-Centric Inertial Hold + AprilTag Tracking
 * 
 * Combines:
 * - Field-centric heading hold (like field-centric drive)
 * - AprilTag tracking (toggled, non-blocking)
 * 
 * Behavior:
 * - Left stick X controls turret rotation (within 20-160° limits)
 * - When stick released, turret HOLDS that field heading even if robot rotates
 * - A toggles AprilTag tracking ON/OFF
 * - When AprilTag tracking is ON, manual control is STILL available
 * (non-blocking)
 * to help the turret if the tag leaves frame
 * 
 * Controls:
 * - Left Stick X: Rotate turret (sets target field heading on release)
 * - A: Toggle AprilTag tracking
 * - D-Pad Down: Reset IMU
 * - Y: Reset turret position to 0°
 * 
 * Limits: 20° to 160°
 */
@TeleOp(name = "TEST: Inclusive Turret", group = "Tests")
public class InclusiveTurretTest extends LinearOpMode {

    private DcMotorEx turret;
    private IMU imu;
    private AprilTagCamera camera;

    // Gear ratio constants (matched to BjornTele)
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 75.52; // Actual ratio (5.23 * 3.61 * 4)
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    // --- Tuned PID Constants (from Auto-Tuner v5) ---
    // --- Improved PID Constants (Balanced) ---
    public static double TURRET_KP = 0.015;
    public static double TURRET_KD = 0.004;
    public static double TURRET_FF = 0.00587;
    public static double ROBOT_ROTATION_FF_GAIN = 0.0055;

    // Deadband: Ignore small errors to reduce jitter
    public static double DEADBAND_DEG = 2.0;

    // Slew Rate: Max power change per loop to smooth output
    public static double MAX_POWER_DELTA = 0.15;

    // Limits
    public static double LIMIT_MIN = 30.0;
    public static double LIMIT_MAX = 155.0;

    // Manual control sensitivity
    private static final double MANUAL_RATE_DEG_PER_SEC = 90.0; // Degrees per second at full stick

    // AprilTag tracking gain (blended with heading hold)
    private static final double APRILTAG_CORRECTION_GAIN = 0.5;

    // State
    private double targetFieldHeading = 90.0; // Default to center of range
    private double turretAngleDeg = 0.0;
    private int lastEncoderPos = 0;
    private double lastPower = 0.0;
    private boolean aprilTagTrackingEnabled = false;
    private boolean wasManualControl = false;

    private boolean aPrev = false;

    @Override
    public void runOpMode() {
        // Initialize turret motor (using BjornConstants for consistency)
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(BjornConstants.Motors.TURRET_DIRECTION);
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

        telemetry.addLine("Inclusive Turret Test");
        telemetry.addLine("Left Stick X: Rotate turret");
        telemetry.addLine("A: Toggle AprilTag tracking");
        telemetry.addLine("D-Pad Down: Reset IMU");
        telemetry.addLine("Y: Reset turret to 0°");
        telemetry.update();

        waitForStart();

        double lastTime = getRuntime();
        double lastError = 0;
        double lastRobotYaw = 0;
        lastEncoderPos = turret.getCurrentPosition();

        while (opModeIsActive()) {
            double now = getRuntime();
            double dt = now - lastTime;
            lastTime = now;

            // Update turret angle from encoder
            int currentPos = turret.getCurrentPosition();
            int deltaTicks = currentPos - lastEncoderPos;
            lastEncoderPos = currentPos;
            turretAngleDeg += (deltaTicks / TURRET_TICKS_PER_DEGREE) * BjornConstants.Motors.TURRET_ENCODER_DIRECTION;

            // Get robot rotation
            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
            double robotYaw = angles.getYaw(AngleUnit.DEGREES);
            double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
            lastRobotYaw = robotYaw;

            // --- Input Handling ---

            // A toggles AprilTag tracking
            if (gamepad1.a && !aPrev) {
                aprilTagTrackingEnabled = !aprilTagTrackingEnabled;
            }
            aPrev = gamepad1.a;

            // D-Pad Down resets IMU
            if (gamepad1.dpad_down) {
                imu.resetYaw();
            }

            // Y resets turret position
            if (gamepad1.y) {
                turretAngleDeg = 0.0;
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lastEncoderPos = 0;
                targetFieldHeading = robotYaw; // Reset target to current robot heading
            }

            // --- Manual Control (Left Stick X) ---
            double stickX = gamepad1.left_stick_x; // Positive: Right stick -> Right turret
            boolean isManualControl = Math.abs(stickX) > 0.1;

            if (isManualControl) {
                // Adjust target heading based on stick input
                double headingDelta = stickX * MANUAL_RATE_DEG_PER_SEC * dt;
                targetFieldHeading += headingDelta;

                // Clamp target to limits (field heading corresponds to turret angle at current
                // robot yaw)
                double minFieldHeading = robotYaw + LIMIT_MIN;
                double maxFieldHeading = robotYaw + LIMIT_MAX;
                targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);

                wasManualControl = true;
            } else if (wasManualControl) {
                // Just released stick - lock to current position
                targetFieldHeading = robotYaw + turretAngleDeg;
                wasManualControl = false;
            }

            // --- AprilTag Tracking (Non-blocking, blended with heading hold) ---
            TagObservation target = null;
            boolean tagVisible = false;

            if (aprilTagTrackingEnabled) {
                List<TagObservation> detections = camera.pollDetections();
                for (TagObservation obs : detections) {
                    if (CameraConfig.isGoalTag(obs.id)) {
                        target = obs;
                        tagVisible = true;
                        break;
                    }
                }

                // If tag visible and NOT doing manual control, adjust target heading
                if (tagVisible && !isManualControl) {
                    // Blend AprilTag correction into target heading
                    // This allows the heading hold to smoothly track the tag
                    targetFieldHeading += target.yaw * APRILTAG_CORRECTION_GAIN * dt * 30.0;

                    // Clamp to limits
                    double minFieldHeading = robotYaw + LIMIT_MIN;
                    double maxFieldHeading = robotYaw + LIMIT_MAX;
                    targetFieldHeading = Range.clip(targetFieldHeading, minFieldHeading, maxFieldHeading);
                }
            }

            // --- Heading Hold Control ---
            double desiredTurretAngle = targetFieldHeading + robotYaw;
            double error = desiredTurretAngle - turretAngleDeg;

            // Apply deadband
            if (Math.abs(error) < DEADBAND_DEG) {
                error = 0;
            }

            // PD Controller
            double derivative = (dt > 0) ? (error - lastError) / dt : 0.0;
            double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
            double ff = robotRate * ROBOT_ROTATION_FF_GAIN;

            double turretPower = pid + ff;
            lastError = error;

            // --- Apply Limits ---
            if (turretAngleDeg < LIMIT_MIN && turretPower < 0)
                turretPower = 0;
            if (turretAngleDeg > LIMIT_MAX && turretPower > 0)
                turretPower = 0;

            // --- Clamp and Slew Rate Limit ---
            if (Double.isNaN(turretPower))
                turretPower = 0;
            turretPower = Range.clip(turretPower, -1.0, 1.0);

            double powerDelta = Range.clip(turretPower - lastPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
            turretPower = lastPower + powerDelta;
            lastPower = turretPower;

            turret.setPower(turretPower * BjornConstants.Motors.TURRET_POWER_DIRECTION);

            // --- Telemetry ---
            telemetry.addData("Mode", isManualControl ? "MANUAL" : "HOLDING");
            telemetry.addData("AprilTag Tracking", aprilTagTrackingEnabled ? "ON" : "OFF");
            if (aprilTagTrackingEnabled) {
                telemetry.addData("Tag Visible", tagVisible);
                if (tagVisible) {
                    telemetry.addData("Tag ID", target.id);
                    telemetry.addData("Tag Yaw", "%.1f°", target.yaw);
                }
            }
            telemetry.addLine();
            telemetry.addData("Robot Yaw", "%.1f°", robotYaw);
            telemetry.addData("Turret Angle", "%.1f°", turretAngleDeg);
            telemetry.addData("Target Heading", "%.1f°", targetFieldHeading);
            telemetry.addData("Error", "%.1f°", error);
            telemetry.addData("Power", "%.2f", turretPower);
            telemetry.addLine();
            telemetry.addData("Limits", "%.0f° - %.0f°", LIMIT_MIN, LIMIT_MAX);
            telemetry.update();
        }

        // Cleanup
        if (camera != null) {
            camera.close();
        }
    }
}
