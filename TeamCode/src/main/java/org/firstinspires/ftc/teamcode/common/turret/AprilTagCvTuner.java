package org.firstinspires.ftc.teamcode.common.turret;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.configurables.CameraConfigurables;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * AprilTag CV Tuner - Camera Optimization & Range Calibration
 * 
 * USES OFFICIAL FTC SDK TAG DATABASE (Into The Deep / DECODE 2024-2025)
 * This ensures correct tag sizes (6.5 inches) for accurate pose estimation.
 * 
 * CONTROLS:
 * - Right Stick Y: Tune Focal Length (Zoom) - Fine-tune Range
 * - LB/LT: Exposure +/-
 * - RB/RT: Gain +/-
 * - D-Pad L/R: Decimation +/-
 * - Y: Reset to Defaults (30fps, FX=930)
 * - X: Save Settings
 * - A: Toggle Live View
 */
@TeleOp(name = "TEST: AprilTag CV Tuner", group = "Tests")
public class AprilTagCvTuner extends OpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Resolution
    private static final Size RESOLUTION = new Size(1280, 720);

    // Initial Intrinsics (used for calibration ratio)
    private double initialFX;

    // Exposure/Gain limits
    private int minExposure = 1;
    private int maxExposure = 100;
    private int minGain = 0;
    private int maxGain = 255;

    // FPS tracking
    private int frameCount = 0;
    private long fpsUpdateTime = 0;
    private double fps = 0.0;

    // Track last applied values
    private int lastAppliedExposure = -1;
    private int lastAppliedGain = -1;
    private int lastAppliedDecimation = -1;

    // Input debounce
    private boolean lastLBump, lastRBump, lastDLeft, lastDRight, lastA, lastY, lastX;
    private boolean lastLTrig, lastRTrig;

    private boolean cameraReady = false;

    @Override
    public void init() {
        // Load saved config into Tuner
        CameraConfigurables.exposure = CameraConfig.TUNED_EXPOSURE;
        CameraConfigurables.gain = CameraConfig.TUNED_GAIN;
        CameraConfigurables.decimation = CameraConfig.TUNED_DECIMATION;
        CameraConfigurables.fx = CameraConfig.FX;
        CameraConfigurables.fy = CameraConfig.FY;

        // Store initial FX for ratio calculation
        initialFX = CameraConfig.FX;

        telemetry.addLine("Initializing AprilTag CV Tuner...");
        telemetry.addLine("Using: FTC SDK Into The Deep Database");
        telemetry.addLine("Camera: Logitech C920 @ 1280x720");
        telemetry.addLine("Place robot at known distance (e.g. 24in)");
        telemetry.update();

        initAprilTag();
    }

    @Override
    public void init_loop() {
        if (!cameraReady) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                getCameraLimits();
                cameraReady = true;
                fpsUpdateTime = System.currentTimeMillis();
            } else {
                telemetry.addLine("Waiting for camera stream...");
                telemetry.update();
                return;
            }
        }

        loopLogic();
    }

    @Override
    public void start() {
        saveSettings();
        fpsUpdateTime = System.currentTimeMillis();
        frameCount = 0;
    }

    @Override
    public void loop() {
        loopLogic();
    }

    private void loopLogic() {
        updateFps();
        applyConfigurableChanges();
        handleInput();
        displayTelemetry();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    private void initAprilTag() {
        // Use the OFFICIAL FTC SDK database for current season (Into The Deep)
        // This has correct tag sizes (6.5 inches) for all field AprilTags
        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(CameraConfig.FX, CameraConfig.FY, CameraConfig.CX, CameraConfig.CY)
                .build();

        aprilTag.setDecimation((float) CameraConfigurables.decimation);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, CameraConfig.WEBCAM_NAME))
                .setCameraResolution(RESOLUTION)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();
    }

    private void getCameraLimits() {
        ExposureControl expCtrl = visionPortal.getCameraControl(ExposureControl.class);
        if (expCtrl != null) {
            minExposure = (int) expCtrl.getMinExposure(TimeUnit.MILLISECONDS) + 1;
            maxExposure = (int) expCtrl.getMaxExposure(TimeUnit.MILLISECONDS);
        }

        GainControl gainCtrl = visionPortal.getCameraControl(GainControl.class);
        if (gainCtrl != null) {
            minGain = gainCtrl.getMinGain();
            maxGain = gainCtrl.getMaxGain();
        }
        applySettings();
    }

    private void applySettings() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)
            return;

        // Apply Exposure
        ExposureControl expCtrl = visionPortal.getCameraControl(ExposureControl.class);
        if (expCtrl != null) {
            if (expCtrl.getMode() != ExposureControl.Mode.Manual) {
                expCtrl.setMode(ExposureControl.Mode.Manual);
            }
            expCtrl.setExposure(CameraConfigurables.exposure, TimeUnit.MILLISECONDS);
        }

        // Apply Gain
        GainControl gainCtrl = visionPortal.getCameraControl(GainControl.class);
        if (gainCtrl != null) {
            gainCtrl.setGain(CameraConfigurables.gain);
        }

        // Apply Decimation
        aprilTag.setDecimation((float) CameraConfigurables.decimation);

        // Update tracking vars
        lastAppliedExposure = CameraConfigurables.exposure;
        lastAppliedGain = CameraConfigurables.gain;
        lastAppliedDecimation = CameraConfigurables.decimation;
    }

    private void applyConfigurableChanges() {
        if (CameraConfigurables.exposure != lastAppliedExposure ||
                CameraConfigurables.gain != lastAppliedGain ||
                CameraConfigurables.decimation != lastAppliedDecimation) {

            CameraConfigurables.exposure = Range.clip(CameraConfigurables.exposure, minExposure, maxExposure);
            CameraConfigurables.gain = Range.clip(CameraConfigurables.gain, minGain, maxGain);
            CameraConfigurables.decimation = Range.clip(CameraConfigurables.decimation, 1, 3);

            applySettings();
        }
    }

    private void saveSettings() {
        CameraConfig.TUNED_EXPOSURE = CameraConfigurables.exposure;
        CameraConfig.TUNED_GAIN = CameraConfigurables.gain;
        CameraConfig.TUNED_DECIMATION = CameraConfigurables.decimation;
        CameraConfig.FX = CameraConfigurables.fx;
        CameraConfig.FY = CameraConfigurables.fy;
    }

    private void updateFps() {
        frameCount++;
        long now = System.currentTimeMillis();
        long elapsed = now - fpsUpdateTime;
        if (elapsed >= 1000) {
            fps = frameCount * 1000.0 / elapsed;
            frameCount = 0;
            fpsUpdateTime = now;
        }
    }

    private void handleInput() {
        // Focal Length (Range Calibration) - Right Stick Y
        float stick = gamepad1.right_stick_y;
        if (Math.abs(stick) > 0.1) {
            double delta = -stick * 10.0;
            CameraConfigurables.fx += delta;
            CameraConfigurables.fy += delta;

            CameraConfigurables.fx = Range.clip(CameraConfigurables.fx, 100.0, 5000.0);
            CameraConfigurables.fy = Range.clip(CameraConfigurables.fy, 100.0, 5000.0);
        }

        // Exposure
        if (gamepad1.left_bumper && !lastLBump)
            CameraConfigurables.exposure++;
        if (gamepad1.left_trigger > 0.25 && !lastLTrig)
            CameraConfigurables.exposure--;

        // Gain
        if (gamepad1.right_bumper && !lastRBump)
            CameraConfigurables.gain += 10;
        if (gamepad1.right_trigger > 0.25 && !lastRTrig)
            CameraConfigurables.gain -= 10;

        // Decimation
        if (gamepad1.dpad_left && !lastDLeft)
            CameraConfigurables.decimation--;
        if (gamepad1.dpad_right && !lastDRight)
            CameraConfigurables.decimation++;

        // Toggle Live View (A)
        if (gamepad1.a && !lastA) {
            if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
                visionPortal.stopStreaming();
            } else {
                visionPortal.resumeStreaming();
            }
        }

        // Reset (Y)
        if (gamepad1.y && !lastY) {
            CameraConfigurables.exposure = 6;
            CameraConfigurables.gain = 250;
            CameraConfigurables.decimation = 3;
            CameraConfigurables.fx = 930.0;
            CameraConfigurables.fy = 930.0;
        }

        // Save (X)
        if (gamepad1.x && !lastX) {
            saveSettings();
        }

        // Update debounces
        lastLBump = gamepad1.left_bumper;
        lastLTrig = gamepad1.left_trigger > 0.25;
        lastRBump = gamepad1.right_bumper;
        lastRTrig = gamepad1.right_trigger > 0.25;
        lastDLeft = gamepad1.dpad_left;
        lastDRight = gamepad1.dpad_right;
        lastA = gamepad1.a;
        lastY = gamepad1.y;
        lastX = gamepad1.x;
    }

    private void displayTelemetry() {
        telemetry.addLine("=== AprilTag CV Tuner ===");
        telemetry.addData("FPS", "%.1f", fps);
        telemetry.addLine("--- Using FTC Database ---");
        telemetry.addData("FX (tune w/ R.Stick Y)", "%.1f", CameraConfigurables.fx);

        double ratio = (initialFX > 0) ? (CameraConfigurables.fx / initialFX) : 1.0;

        List<AprilTagDetection> detections = aprilTag.getDetections();

        int goalTags = 0;
        AprilTagDetection bestGoal = null;
        for (AprilTagDetection det : detections) {
            if (CameraConfig.isGoalTag(det.id)) {
                goalTags++;
                if (bestGoal == null
                        || (det.ftcPose != null && Math.abs(det.ftcPose.z) < Math.abs(bestGoal.ftcPose.z))) {
                    bestGoal = det;
                }
            }
        }

        if (bestGoal != null && bestGoal.ftcPose != null) {
            double x = bestGoal.ftcPose.x;
            double y = bestGoal.ftcPose.y;
            double z = bestGoal.ftcPose.z;

            // TRUE RANGE = Euclidean distance (accounts for tilted camera)
            double trueRange = Math.sqrt(x * x + y * y + z * z);

            telemetry.addData("Tag", "ID %d (%s)", bestGoal.id,
                    bestGoal.id == CameraConfig.BLUE_GOAL_TAG_ID ? "BLUE" : "RED");
            telemetry.addData(">>> TRUE RANGE <<<", "%.1f in", trueRange);
            telemetry.addData("Z (forward)", "%.1f in", z);
            telemetry.addData("Yaw", "%.1f deg", bestGoal.ftcPose.yaw);
            telemetry.addData("XYZ", "x=%.1f y=%.1f z=%.1f", x, y, z);
        } else {
            telemetry.addLine("NO GOAL TAG (ID 20/24) DETECTED");
            telemetry.addData("Total detections", detections.size());
            for (AprilTagDetection det : detections) {
                if (det.ftcPose != null) {
                    double tr = Math.sqrt(det.ftcPose.x * det.ftcPose.x + det.ftcPose.y * det.ftcPose.y
                            + det.ftcPose.z * det.ftcPose.z);
                    telemetry.addData("ID " + det.id, "range=%.1f", tr);
                }
            }
        }

        telemetry.addLine("--- Settings ---");
        telemetry.addData("Exp/Gain/Dec", "%d / %d / %d",
                CameraConfigurables.exposure, CameraConfigurables.gain, CameraConfigurables.decimation);

        boolean saved = (CameraConfigurables.exposure == CameraConfig.TUNED_EXPOSURE &&
                CameraConfigurables.fx == CameraConfig.FX);
        telemetry.addData("Status", saved ? "SAVED" : "UNSAVED (X to save)");

        telemetry.update();
    }
}
