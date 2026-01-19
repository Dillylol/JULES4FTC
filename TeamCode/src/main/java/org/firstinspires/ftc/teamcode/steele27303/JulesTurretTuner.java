package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera.TagObservation;

import java.util.List;
import java.util.Locale;

@TeleOp(name = "JulesTurretTuner", group = "Test")
public class JulesTurretTuner extends LinearOpMode {

    private BjornHardware hardware;
    private AprilTagCamera aprilTagCamera;
    
    // Modes
    private enum TrackMode {
        A_ENCODER_ONLY,
        X_CAMERA_ONLY,
        B_FUSED_JULES
    }
    
    private TrackMode currentMode = TrackMode.A_ENCODER_ONLY;
    private boolean activeControlEnabled = false; // Toggled by D-Pad Down
    
    // Logic States
    private boolean dpadDownPrev = false;
    
    // Tuning Constants (Hardcoded for now, could be @Config)
    private static final double ACTIVE_P_GAIN = 0.025; // Power per degree of error
    private static final double MANUAL_POWER_SCALAR = 0.4;
    
    @Override
    public void runOpMode() {
        hardware = BjornHardware.forTeleOp(hardwareMap);
        
        // Initialize Camera
        aprilTagCamera = new AprilTagCamera();
        aprilTagCamera.start(hardwareMap, null);
        
        // Reset Estimator to 0 (assuming we start aligned-ish or reset happens)
        hardware.turretEstimator.reset(0.0, 1.0); // low uncertainty to start
        
        telemetry.addLine("JULES Turret Tuner Ready");
        telemetry.addLine("A: Encoder Mode | X: Camera Mode | B: Fused Mode");
        telemetry.addLine("D-Pad Down: Toggle Active Tracking");
        telemetry.addLine("D-Pad L/R: Manual Control");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            hardware.updateEstimator(); // Run Prediction Step (Encoder)
            
            // --- Input Handling ---
            if (gamepad1.a) currentMode = TrackMode.A_ENCODER_ONLY;
            if (gamepad1.x) currentMode = TrackMode.X_CAMERA_ONLY;
            if (gamepad1.b) currentMode = TrackMode.B_FUSED_JULES;
            
            boolean dpadDown = gamepad1.dpad_down;
            if (dpadDown && !dpadDownPrev) {
                activeControlEnabled = !activeControlEnabled;
                // If enabling active control, maybe reset PID integrator if we had one?
            }
            dpadDownPrev = dpadDown;
            
            // --- Camera Update ---
            TagObservation bestBlueTag = null;
            List<TagObservation> detections = aprilTagCamera.pollDetections();
            for (TagObservation obs : detections) {
                // Filter for Blue Goal Tag
                if (obs.id == org.firstinspires.ftc.teamcode.common.CameraConfig.BLUE_GOAL_TAG_ID) {
                    if (bestBlueTag == null || Math.abs(obs.yaw) < Math.abs(bestBlueTag.yaw)) {
                        bestBlueTag = obs;
                    }
                }
            }
            aprilTagCamera.publishDetections();
            
            double cameraErrorDeg = (bestBlueTag != null) ? bestBlueTag.yaw : Double.NaN;
            
            // Note: Camera Yaw is usually Positive = Right (Tag is to the right).
            // Turret Angle definition: Positive = Right ??
            // If Turret is Right of Tag (+), Camera sees Tag Left (-).
            // So Observed Angle = -Yaw. 
            // We pass this to corrected belief.
            
            if (bestBlueTag != null) {
                double observedAngle = -bestBlueTag.yaw; // Check sign convention! 
                
                if (currentMode == TrackMode.B_FUSED_JULES) {
                    hardware.correctTurretBelief(observedAngle);
                }
            }
            
            // --- Control Logic ---
            double motorPower = 0.0;
            double manualCmd = 0.0;
            
            if (gamepad1.dpad_left) manualCmd = -MANUAL_POWER_SCALAR;
            if (gamepad1.dpad_right) manualCmd = MANUAL_POWER_SCALAR;
            
            double errorForControl = 0.0;
            boolean hasValidError = false;
            
            switch (currentMode) {
                case A_ENCODER_ONLY:
                    // Manual only. Belief is strictly encoder dead-reckoning.
                    // Active toggle does nothing (or maybe resets to 0?) -> User said "dpad active" toggles active control x/b.
                    // Re-reading: "a is encoder-based tracking... dpad for moving angle". Active toggle not explicitly mentioned for A.
                    // We'll leave A as pure manual.
                    motorPower = manualCmd;
                    break;
                    
                case X_CAMERA_ONLY:
                    // "Camera based tracking... toggle active control... guidance to the camera"
                    // "Do not use pure encoder data"
                    if (bestBlueTag != null) {
                        errorForControl = -bestBlueTag.yaw; // Using raw camera error
                        hasValidError = true;
                    }
                    
                    if (activeControlEnabled && hasValidError) {
                         // Simple P-Control to zero the error
                         // Target is 0. Error = Target - Observed. 0 - (Observed).
                         // Power = Kp * (0 - error) = -Kp * Error.
                         // Check signs: If error is +10 (Turret is Right), we want Left Power (-). 
                         // So Power = -Kp * Error.
                         motorPower = -ACTIVE_P_GAIN * errorForControl;
                    }
                    
                    // User said: "left and right dpad still manually control" (for Mode B, presumably X too per "like the camera one")
                    motorPower += manualCmd; 
                    break;
                    
                case B_FUSED_JULES:
                    // Uses Estimator Belief.
                    // Target is 0 (Aligned). Belief explains where we are.
                    double belief = hardware.turretEstimator.getBelief();
                    
                    if (activeControlEnabled) {
                        // Error = Belief. We want Belief -> 0.
                        // If Belief = +10, Power should be -.
                        motorPower = -ACTIVE_P_GAIN * belief;
                    }
                    
                    motorPower += manualCmd; 
                    break;
            }
            
            // Safety Clamp
            motorPower = Range.clip(motorPower, -1.0, 1.0);
            
            if (hardware.turret != null) {
                hardware.turret.setPower(motorPower);
            }
            
            // --- Telemetry ---
            telemetry.addLine("=== JULES Turret Tuner ===");
            telemetry.addData("Mode", currentMode);
            telemetry.addData("Active", activeControlEnabled);
            telemetry.addData("Power", String.format(Locale.US, "%.2f", motorPower));
            telemetry.addData("Tag Visible", (bestBlueTag != null));
            
            telemetry.addLine("\n--- Data ---");
            telemetry.addData("Encoder (Deg)", String.format(Locale.US, "%.2f", hardware.turretEstimator.getBelief())); // For A/B this is main belief
            telemetry.addData("Camera Err (Deg)", hasValidError ? String.format(Locale.US, "%.2f", errorForControl) : "No Tag");
            
            if (currentMode == TrackMode.B_FUSED_JULES) {
                 telemetry.addLine("\n--- JULES State ---");
                 telemetry.addData("Belief (x)", String.format(Locale.US, "%.2f", hardware.turretEstimator.getBelief()));
                 telemetry.addData("Uncertainty (P)", String.format(Locale.US, "%.4f", hardware.turretEstimator.getUncertainty()));
                 telemetry.addData("Conflict", String.format(Locale.US, "%.2f", hardware.turretEstimator.getConflict()));
                 telemetry.addData("Arguing", hardware.turretEstimator.isArguing());
            }
            
            telemetry.update();
        }
    }
}
