package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.common.turret.TurretEstimator;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;

/**
 * JULES Turret Gunner Test
 * 
 * Tests the "Turret Belief Engine" (TurretEstimator) separately from full TeleOp.
 * 
 * Controls (Gamepad 2 - Gunner):
 * - Y: Toggle Manual Override (Direct stick control, no PID/Estimator influence)
 * - A: Toggle JULES Prediction/Tracking (Uses Estimator + AprilTags)
 * - D-Pad Down: Reset IMU & Belief State
 * - Left Stick X: Manual Turret (when in Manual Mode)
 */
@TeleOp(name = "JULES Turret Gunner Test", group = "Test")
public class BjornTurretTest extends BjornTeleBase {

    // --- Hardware ---
    private DcMotorEx turret;
    private IMU imu;
    private AprilTagCamera aprilTagCamera;
    
    // --- JULES Belief Engine ---
    private TurretEstimator turretEstimator;
    
    // --- Constants ---
    private static final int GOAL_TAG_ID = CameraConfig.BLUE_GOAL_TAG_ID; // Default to Blue for test
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 75.52; 
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;
    
    // PIDF (Same as TeleOp Round 2)
    private static final double TURRET_KP = 0.024;
    private static final double TURRET_KD = 0.0055;
    private static final double ROBOT_ROTATION_FF_GAIN = 0.0055;
    private static final double DEADBAND_DEG = 2.0;
    private static final double MAX_POWER_DELTA = 0.15;
    
    // Limits
    private static final double LIMIT_MIN = 20.0;
    private static final double LIMIT_MAX = 160.0;
    private static final double MANUAL_POWER = 0.5;

    // --- State ---
    private double targetFieldHeading = 90.0;
    private int lastEncoderPos = 0;
    private double lastTurretPower = 0.0;
    private double lastTurretError = 0.0;
    private double lastRobotYaw = 0.0;
    private double lastLoopTime = 0;

    // Modes
    private boolean manualOverride = false; // Y toggles
    private boolean julesTrackingEnabled = false; // A toggles

    // Toggles
    private boolean yPrev = false;
    private boolean aPrev = false;

    @Override
    protected String getAllianceName() {
        return "TEST";
    }

    @Override
    protected int getGoalTagId() {
        return GOAL_TAG_ID;
    }

    @Override
    public void init() {
        // Init Turret
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(BjornConstants.Motors.TURRET_DIRECTION);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Init IMU
        imu = hardwareMap.get(IMU.class, BjornConstants.Sensors.IMU);
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        // Init Camera
        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);

        // Init Belief Engine
        // Start with 0 degrees belief, 0 uncertainty (we just reset encoders)
        turretEstimator = new TurretEstimator(0.0, 0.0);

        // Init other subsystems (Grips, Shooter) just in case
        initSubsystems();

        telemetry.addLine("JULES Turret Test Initialized");
        telemetry.addLine("Controls: Y=Manual, A=Track, DPadDn=Reset");
        telemetry.update();
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        double now = nowMs / 1000.0;
        double dt = (lastLoopTime > 0) ? (now - lastLoopTime) : 0.02;
        lastLoopTime = now;

        // --- Inputs ---
        handleInput();

        // --- JULES Engine ---
        updateBelief(dt);

        // --- Control ---
        if (manualOverride) {
            runManualControl();
        } else {
            runJulesControl(dt);
        }

        // --- Telemetry ---
        telemetry.addData("Mode", manualOverride ? "MANUAL (Y)" : "JULES (A to Track)");
        telemetry.addData("Tracking", julesTrackingEnabled ? "ON" : "OFF");
        telemetry.addData("Turret Angle", "%.1f°", turretEstimator.getBelief());
        telemetry.addData("Target Heading", "%.1f°", targetFieldHeading);
        telemetry.addData("Uncertainty (P)", "%.3f", turretEstimator.getUncertainty());
        telemetry.addData("Conflict", "%.1f°", turretEstimator.getConflict());
        telemetry.addData("Anomalies", turretEstimator.isArguing() ? "DETECTED!" : "None");
        
        telemetry.update();
    }

    private void handleInput() {
        // Y: Toggle Manual
        if (gamepad2.y && !yPrev) {
            manualOverride = !manualOverride;
            // When entering manual, disable tracking
            if (manualOverride) julesTrackingEnabled = false;
        }
        yPrev = gamepad2.y;

        // A: Toggle Tracking (Only if not in manual)
        if (gamepad2.a && !aPrev && !manualOverride) {
            julesTrackingEnabled = !julesTrackingEnabled;
        }
        aPrev = gamepad2.a;

        // D-Pad Down: Reset
        if (gamepad2.dpad_down) {
            imu.resetYaw();
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretEstimator.reset(0.0, 0.0);
            targetFieldHeading = 0.0;
            turretEstimator = new TurretEstimator(0.0, 0.0); // Hard reset
            lastEncoderPos = 0;
        }
    }

    private void updateBelief(double dt) {
        // 1. Prediction (Encoder)
        int currentPos = turret.getCurrentPosition();
        int deltaTicks = currentPos - lastEncoderPos;
        lastEncoderPos = currentPos;
        
        double deltaDeg = deltaTicks / TURRET_TICKS_PER_DEGREE;
        turretEstimator.predict(deltaDeg);

        // 2. Update (Camera)
        if (julesTrackingEnabled) {
            List<TagObservation> detections = aprilTagCamera.pollDetections();
            TagObservation target = null;
            for (TagObservation obs : detections) {
                if (obs.id == GOAL_TAG_ID) {
                    target = obs;
                    break;
                }
            }

            if (target != null) {
                // Determine observed turret angle from tag
                // Goal is at 90 deg field heading (roughly).
                // But simplified: The tag gives us Relative Yaw from Camera.
                // Camera is on Turret? Or Robot?
                // Standard setup: Camera is fixed on Robot or Turret?
                // Wait, if Camera is on Turret: Tag Yaw = 0 means we are looking at it.
                // If Camera is on Robot (Fixed): We can't see turret angle directly.
                
                // ASSUMPTION: Camera is on ROBOT (Fixed) based on previous TeleOp logic 
                // "targetFieldHeading -= target.yaw" -> changing Field Target based on Robot-Ref tag.
                // Wait, TeleOp logic says: "targetFieldHeading -= target.yaw".
                // If camera is on Turret, and we center tag, Yaw -> 0.
                
                // Let's look at TurretEstimator.correction(observedAngle).
                // It expects absolute angle.
                // If we can't observe absolute angle (no pot), we can't correct *state* x.
                // We can only correct *error*.
                
                // RE-READING ARCHITECTURE:
                // "Update step fuses Derived State from telemetry".
                // If we don't have an absolute sensor, we can't Update Position X.
                // WE CAN only use Tag to change TARGET.
                
                // HOWEVER, user asked to "test turret belief engine".
                // Maybe we just use Encoder for Belief, and Tag modifies Target (as in TeleOp).
                // Or maybe we use Tag to correct Drift? (Only if we know Tag's Field Position + Robot Field Position).
                
                // For this test, let's keep it simple and aligned with TeleOp:
                // Tracking changes TARGET, estimator tracks STATE.
                // BUT, if we want to "Update" the estimator, we need a measurement.
                // We don't have one without absolute encoders or knowing exact Robot Yaw + Tag Field Pos.
                
                // Let's implement the standard TeleOp tracking for "JULES Prediction":
                // Camera -> Adjusts Target Field Heading.
                // Estimator -> Tracks where Turret IS.
                // PID -> Moves Estimator.Belief towards Target.
                
                // The "Prediction" part of user request: "camera to predict where tag will be"
                // That might imply Kalman filtering the TAG position? 
                // "rotate the turret accordingly" using "JULES prediction engine".
                
                // Since I only have TurretEstimator (1D Scalar), I will use it to track Turret Angle.
                // I will use Tag to update Target.
                
                // Logic from TeleOp: targetFieldHeading -= tag.yaw ...
                targetFieldHeading += target.yaw * 0.5 * dt * 30.0 * CameraConfig.CAMERA_TO_TURRET_SCALAR; // Correction gain
            }
        }
    }

    private void runManualControl() {
        double power = -gamepad2.left_stick_x * MANUAL_POWER;
        turret.setPower(power);
        targetFieldHeading = turretEstimator.getBelief() + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    private void runJulesControl(double dt) {
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double robotRate = (dt > 0) ? (robotYaw - lastRobotYaw) / dt : 0.0;
        lastRobotYaw = robotYaw;

        // Target (Field) - Robot (Field) = Desired Turret (Relative)
        double desiredTurretAngle = targetFieldHeading - robotYaw;
        
        // Error = Goal - Belief
        double currentBelief = turretEstimator.getBelief();
        double error = desiredTurretAngle - currentBelief;

        if (Math.abs(error) < DEADBAND_DEG) error = 0;

        double derivative = (dt > 0) ? (error - lastTurretError) / dt : 0.0;
        double pid = (error * TURRET_KP) + (derivative * TURRET_KD);
        double ff = -robotRate * ROBOT_ROTATION_FF_GAIN;

        double power = pid + ff;
        lastTurretError = error;

        // Bounds check (Soft Stops using Belief)
        if (currentBelief < LIMIT_MIN && power < 0) power = 0;
        if (currentBelief > LIMIT_MAX && power > 0) power = 0;

        // Slew Rate
        double powerDelta = Range.clip(power - lastTurretPower, -MAX_POWER_DELTA, MAX_POWER_DELTA);
        power = lastTurretPower + powerDelta;
        lastTurretPower = power;

        turret.setPower(power);
    }
}
