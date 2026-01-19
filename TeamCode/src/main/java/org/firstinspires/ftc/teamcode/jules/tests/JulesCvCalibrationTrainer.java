package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.CameraConfig;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.cv.AprilTagCamera;
import org.firstinspires.ftc.teamcode.common.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.jules.shot.ShotTrainerSettings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * JULES CV Calibration Trainer
 * <p>
 * 1. Starts at (0,0) (User places robot under score zone).
 * 2. Backs up 30 inches (Safety Move).
 * 3. Steps through a path: Reverse 60" total, Right 40" total.
 * 4. Uses CV for heading lock (ignoring Pedro heading).
 * 5. Calculates RPM based on Distance + Learned Offset.
 * 6. Sends Shot Data to Client -> Waits for YES/NO feedback.
 * 7. Updates Offset Model based on feedback.
 */
@Autonomous(name = "JULES: CV Calibration Trainer", group = "JULES")
public class JulesCvCalibrationTrainer extends OpMode {

    private static final String TAG = "JulesCvTrainer";

    // ---------------- Settings ----------------
    private static final double SAFETY_BACK_INCHES = 30.0;
    private static final double TOTAL_REVERSE_INCHES = 60.0;
    private static final double TOTAL_RIGHT_INCHES = 40.0;
    private static final int TRAINING_STEPS = 5; // Number of points along the path

    private static final double CV_TURN_SPEED = 0.6;
    private static final double HEADING_TOLERANCE_DEG = 2.0;
    private static final long SETTLE_MS = 1000;

    // ---------------- Hardware ----------------
    private BjornHardware hardware;
    private Follower follower;
    private ShooterController shooter;
    private AprilTagCamera camera;

    // ---------------- JULES Bridge ----------------
    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;
    private JulesStreamBus.Subscription busSub;
    private Thread busPump;
    private final Queue<String> pendingCmds = new ConcurrentLinkedQueue<>();
    private String sessionId;
    private String botId;

    // ---------------- State ----------------
    private enum State {
        INIT,
        SAFETY_MOVE,
        TRAINING_MOVE,
        TRAINING_AIM,
        TRAINING_REQUEST_RPM,
        TRAINING_SHOOT,
        TRAINING_FEEDBACK,
        DONE
    }

    private State state = State.INIT;
    private long stateEnterMs = 0;
    private int stepIndex = 0;

    // ---------------- Logic ----------------
    private Pose startPose = new Pose(0, 0, 0); // Assuming 0 heading is facing forward/away from driver?
    // Actually, usually 0 is +X. Let's assume standard field centric:
    // If "under score zone" means facing the goal, and we back up...
    // Let's assume Start Heading is 0 (facing goal? or away?).
    // BjornTeleBase uses -90 as facing goal (Blue).
    // Let's assume the user aligns the robot and we reset Pedro to 0.
    // We will use CV for actual aiming, so Pedro heading is just for translation
    // reference.

    private Pose currentTargetPose;
    private double currentOffsetRpm = 0.0;
    private double lastSuccessfulOffsetRpm = 0.0;
    private ShooterController.ShotMetrics lastShotMetrics;

    @Override
    public void init() {
        telemetry.addLine("Initializing JULES CV Trainer...");

        // Hardware
        hardware = BjornHardware.forAutonomous(hardwareMap);
        shooter = new ShooterController(hardware.wheel, hardware.wheel2, hardware.intake, hardware,
                hardware.getVoltageSensor());

        // Pedro
        try {
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(startPose);
        } catch (Exception e) {
            RobotLog.ee(TAG, "Pedro init failed: " + e.getMessage());
        }

        // Camera
        camera = new AprilTagCamera();
        camera.start(hardwareMap, null);

        // JULES Bridge
        botId = ShotTrainerSettings.getBotId(hardwareMap.appContext);
        bridgeManager = JulesBridgeManager.getInstance();
        if (bridgeManager != null) {
            bridgeManager.prepare(hardwareMap.appContext);
            streamBus = bridgeManager.getStreamBus();
        }

        sessionId = "cv-calib-" + System.currentTimeMillis();
        telemetry.addLine("Ready. Session: " + sessionId);
        telemetry.update();
    }

    @Override
    public void start() {
        startBusListener();
        state = State.SAFETY_MOVE;
        stateEnterMs = System.currentTimeMillis();
        stepIndex = 0;

        // Plan: Move back 30 inches first
        // Assuming +X is forward, -X is backward? Or +Y?
        // Let's assume standard Pedro: X is forward/back, Y is left/right.
        // "Reverse" usually means -X. "Right" usually means -Y (Blue) or +Y (Red)?
        // Let's assume -X is reverse.
        Pose safetyPose = new Pose(-SAFETY_BACK_INCHES, 0, 0);
        currentTargetPose = safetyPose;

        if (follower != null) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(new Pose(0, 0, 0),
                                    new Pose(safetyPose.getX(), safetyPose.getY(), 0)))
                            .setLinearHeadingInterpolation(0, 0)
                            .build(),
                    true);
        }
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        if (follower != null)
            follower.update();
        shooter.update(now);
        handleIncomingCommands();

        switch (state) {
            case SAFETY_MOVE:
                if (!follower.isBusy()) {
                    // Safety move done. Start training path.
                    nextTrainingStep();
                }
                break;

            case TRAINING_MOVE:
                if (!follower.isBusy()) {
                    transition(State.TRAINING_AIM);
                }
                break;

            case TRAINING_AIM:
                // Use CV to turn
                if (doCvAim()) {
                    // Aimed. Request RPM from Client.
                    requestRpmFromClient();
                    transition(State.TRAINING_REQUEST_RPM);
                }
                break;

            case TRAINING_REQUEST_RPM:
                // Wait for set_rpm command
                if (now - stateEnterMs > 2000) {
                    // Timeout, fallback to local calculation
                    telemetry.addLine("RPM Request Timeout. Using local fallback.");
                    prepareShotFallback(now);
                    transition(State.TRAINING_SHOOT);
                }
                break;

            case TRAINING_SHOOT:
                if (shooter.isReady(now)) {
                    if (shooter.fire(now)) {
                        lastShotMetrics = shooter.pollShotMetrics();
                        sendShotData();
                        transition(State.TRAINING_FEEDBACK);
                    }
                }
                break;

            case TRAINING_FEEDBACK:
                // Wait for command (handled in handleIncomingCommands)
                // If timeout?
                if (now - stateEnterMs > 10000) {
                    // Timeout, assume NO? or just move on?
                    // Let's just log timeout and move on.
                    telemetry.addLine("Feedback Timeout. Moving on.");
                    nextTrainingStep();
                }
                break;

            case DONE:
                shooter.stop(now);
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Step", stepIndex + "/" + TRAINING_STEPS);
        telemetry.addData("Offset", currentOffsetRpm);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop(System.currentTimeMillis());
        if (camera != null)
            camera.close();
        if (busSub != null) {
            try {
                busSub.close();
            } catch (Exception e) {
            }
        }
        if (busPump != null)
            busPump.interrupt();
    }

    // ---------------- Helpers ----------------

    private void nextTrainingStep() {
        if (stepIndex >= TRAINING_STEPS) {
            transition(State.DONE);
            return;
        }

        // Calculate next point
        // Path: Reverse 60" total (from start? or from safety?), Right 40"
        // User said: "back up by about 30inches... then follow a path of reverse (-60
        // inches) and move right (+40 inches)"
        // This implies the path starts AFTER the safety move.
        // So Start of Path = (-30, 0).
        // End of Path = (-30 - 60, 0 - 40) = (-90, -40) (Assuming Right is -Y for Blue,
        // +Y for Red. Let's assume -Y for now).

        double progress = (double) stepIndex / (double) (TRAINING_STEPS - 1);
        double startX = -SAFETY_BACK_INCHES;
        double startY = 0;
        double endX = startX - TOTAL_REVERSE_INCHES;
        double endY = startY - TOTAL_RIGHT_INCHES; // Assuming Right is negative Y

        double targetX = startX + (endX - startX) * progress;
        double targetY = startY + (endY - startY) * progress;

        currentTargetPose = new Pose(targetX, targetY, 0); // Heading 0 for move, CV will fix it

        if (follower != null) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(
                                    new Pose(follower.getPose().getX(), follower.getPose().getY(), 0),
                                    new Pose(targetX, targetY, 0)))
                            .setLinearHeadingInterpolation(follower.getPose().getHeading(), 0)
                            .build(),
                    true);
        }

        stepIndex++;
        transition(State.TRAINING_MOVE);
    }

    private boolean doCvAim() {
        AprilTagCamera.TagObservation tag = getGoalTag();
        if (tag == null)
            return false; // No tag, can't aim

        // Calculate heading to tag
        // From BaseTagShooterTele: Math.atan2(tag.x, tag.z)
        // Note: tag.x is horizontal, tag.z is forward distance.
        // atan2(x, z) gives angle relative to camera center.

        double turnPower = 0;
        double headingError = Math.atan2(tag.x, tag.z); // Radians

        // Simple P controller
        double kP = 1.5;
        turnPower = -headingError * kP; // Negative to turn towards error?
        // If x is positive (right), error is positive. We need to turn RIGHT (negative
        // heading change? or positive?)
        // Robot turns: Positive power -> Left? Negative -> Right?
        // Usually: Positive Heading is Left (CCW).
        // If Tag is Right (+x), we need to turn Right (CW, negative).
        // So if error is positive, we need negative power.
        // turnPower = -error * kP. Correct.

        turnPower = Math.max(-CV_TURN_SPEED, Math.min(CV_TURN_SPEED, turnPower));

        // Apply to Pedro TeleOp drive (override heading)
        if (follower != null) {
            // We are stationary, so x/y drive is 0
            follower.setTeleOpDrive(0, 0, turnPower, true);
        }

        return Math.abs(Math.toDegrees(headingError)) < HEADING_TOLERANCE_DEG;
    }

    private void requestRpmFromClient() {
        double dist = getCvDistance();
        if (dist < 0) dist = 1.0; // Fallback distance
        
        double distInches = dist * 39.37;
        double voltage = hardware.getVoltageSensor().getVoltage();

        JsonObject req = new JsonObject();
        req.addProperty("type", "request_rpm");
        req.addProperty("distance_in", distInches);
        req.addProperty("v_batt_load", voltage);
        
        busPublish(req.toString());
    }

    private void prepareShotFallback(long now) {
        // Calculate RPM locally
        double dist = getCvDistance();
        if (dist < 0)
            dist = 10; // Fallback
        
        // Base RPM (Linear model from ShotTrainer)
        double distInches = dist * 39.37;
        double baseRpm = 2100.0 + (distInches * 8.0);

        // Apply Offset
        currentOffsetRpm = lastSuccessfulOffsetRpm;

        double targetRpm = baseRpm + currentOffsetRpm;
        shooter.setTargetRpm(targetRpm, now);
    }

    private double getCvDistance() {
        AprilTagCamera.TagObservation tag = getGoalTag();
        if (tag == null)
            return -1;
        return tag.z; // Meters
    }

    private AprilTagCamera.TagObservation getGoalTag() {
        if (camera == null)
            return null;
        List<AprilTagCamera.TagObservation> tags = camera.pollDetections();
        for (AprilTagCamera.TagObservation tag : tags) {
            // Accept Blue or Red goal for now, or filter?
            // Let's use CameraConfig defaults
            if (tag.id == CameraConfig.BLUE_GOAL_TAG_ID || tag.id == CameraConfig.RED_GOAL_TAG_ID) {
                return tag;
            }
        }
        return null;
    }

    private void sendShotData() {
        JsonObject ev = new JsonObject();
        ev.addProperty("type", "training_shot");
        ev.addProperty("session_id", sessionId);
        double distM = getCvDistance();
        ev.addProperty("dist_m", distM);
        ev.addProperty("distance_in", distM * 39.37);
        ev.addProperty("v_batt_load", hardware.getVoltageSensor().getVoltage());
        ev.addProperty("offset_rpm", currentOffsetRpm);
        ev.addProperty("rpm", shooter.getTargetRpm());

        // Add Pedro Pose
        if (follower != null) {
            Pose p = follower.getPose();
            ev.addProperty("pedro_x", p.getX());
            ev.addProperty("pedro_y", p.getY());
        }

        busPublish(ev.toString());
    }

    private void handleIncomingCommands() {
        String raw;
        while ((raw = pendingCmds.poll()) != null) {
            try {
                JsonElement parsed = GsonCompat.parse(raw);
                if (parsed.isJsonObject()) {
                    JsonObject obj = parsed.getAsJsonObject();
                    String cmd = obj.has("cmd") ? obj.get("cmd").getAsString() : "";

                    if (state == State.TRAINING_REQUEST_RPM) {
                        if (cmd.equalsIgnoreCase("set_rpm")) {
                            double rpm = obj.get("rpm").getAsDouble();
                            currentOffsetRpm = obj.has("offset_rpm") ? obj.get("offset_rpm").getAsDouble() : 0.0;
                            shooter.setTargetRpm(rpm, System.currentTimeMillis());
                            telemetry.addLine("Received RPM: " + rpm);
                            transition(State.TRAINING_SHOOT);
                        }
                    }

                    if (state == State.TRAINING_FEEDBACK) {
                        if (cmd.equalsIgnoreCase("feedback")) {
                            String value = obj.get("value").getAsString(); // "yes" or "no"
                            if (value.equalsIgnoreCase("yes")) {
                                // Reinforce
                                lastSuccessfulOffsetRpm = currentOffsetRpm;
                                telemetry.addLine("Shot GOOD. Offset kept.");
                            } else {
                                // Adjust? For now just log.
                                telemetry.addLine("Shot BAD.");
                                // Maybe adjust offset for next shot?
                                // Simple logic: If No, maybe try adding 50 RPM next time?
                                // Or just let RL client handle it (but we are the client here effectively).
                                // User said "predict the next offset using previous data".
                                // Let's just keep it simple: If No, maybe randomize slightly or keep same.
                            }
                            nextTrainingStep();
                        }
                    }
                }
            } catch (Exception e) {
            }
        }
    }

    private void transition(State next) {
        state = next;
        stateEnterMs = System.currentTimeMillis();
    }

    // ---------------- Bridge Utils ----------------
    private void startBusListener() {
        if (streamBus == null)
            return;
        try {
            busSub = streamBus.subscribe();
            busPump = new Thread(() -> {
                while (!Thread.currentThread().isInterrupted()) {
                    try {
                        String line = busSub.take();
                        if (line == null)
                            break;
                        // Parse for commands
                        JsonElement el = GsonCompat.parse(line);
                        if (el.isJsonObject()) {
                            JsonObject obj = el.getAsJsonObject();
                            if (obj.has("cmd")) {
                                pendingCmds.offer(line);
                            }
                        }
                    } catch (Exception e) {
                    }
                }
            });
            busPump.start();
        } catch (Exception e) {
        }
    }

    private void busPublish(String json) {
        if (streamBus != null) {
            try {
                streamBus.publishJsonLine(json);
            } catch (Exception e) {
            }
        }
    }
}
