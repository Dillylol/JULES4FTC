package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.jules.shot.ShotTrainerSettings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Pedro-driven shot trainer that walks the robot through a set of ranges and reports shots on the
 * primary JULES telemetry stream so the desktop RL client can score them.
 */
@Autonomous(name = "JULES: Shot Trainer", group = "JULES")
public final class ShotTrainerOpMode extends OpMode {

    private static final String TAG = "ShotTrainerOpMode";

    // Distance band for training.
    private static final double MIN_RANGE_IN = 20.0;
    private static final double MAX_RANGE_IN = 75.0;
    private static final double STEP_IN = 8.0;
    private static final double START_RANGE_IN = 25.0;

    // Pedro tolerances for "at range".
    private static final double RANGE_TOLERANCE_IN = 2;
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3.0);

    // 0" = closest legal shooting line along -X. Positive range = farther from goal.
    // Heading is still 0 rad; adjust if you want the bot rotated differently.
    private static final double TRAIN_HEADING_RAD = Math.toRadians(0.0);

    // Time to wait after a shot before stopping shooter/intake and advancing range.
    // This effectively controls how long the intake stays on after a fire.
    // Tweak this as needed (currently ~3 seconds).
    private static final long WAIT_FOR_REWARD_MS = 3_000L;

    private static final double RPM_MIN = 1_800.0;
    private static final double RPM_MAX = 3_000.0;
    private static final double MAX_OVERRIDE_DELTA_RPM = 250.0;

    // Real pack sensor (if present) – initialized like the virtual shooter
    private VoltageSensor voltageSensor;

    private ShooterController shooter;
    private Follower drive;
    private Pose basePoseNearGoal = new Pose(0.0, 0.0, TRAIN_HEADING_RAD);
    private Pose pendingTargetPose;

    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;
    private JulesStreamBus.Subscription busSub;
    private Thread busPump;
    private final Queue<String> pendingCmds = new ConcurrentLinkedQueue<>();

    private double currentRangeIn = START_RANGE_IN;
    private int rangeDirection = +1;
    private double currentTargetRpm = 0.0;
    private double currentBaselineRpm = 0.0;
    private double currentOverrideRpm = Double.NaN;
    private double currentBatteryV = 12.0;

    private ShooterController.ShotMetrics lastShotMetrics;
    private long lastShotTimestampMs;
    private long shotSeq = 0L;
    private String sessionId = newSessionId();
    private String botId = "shot-trainer";

    private TrainState state = TrainState.INIT_POSITION;
    private long stateSinceMs = 0L;

    private enum TrainState {
        INIT_POSITION,
        DRIVING_TO_RANGE,
        SPINNING_UP,
        READY_TO_FIRE,
        FIRING,
        WAITING_REWARD,
        ADVANCING_RANGE,
        DONE
    }

    private enum RpmControlMode {
        BASELINE,
        LEARNED
    }

    // TODO: expose RPM mode selection via dashboard / DS control when RL client is ready.
    private RpmControlMode rpmControlMode = RpmControlMode.BASELINE;

    @Override
    public void init() {
        telemetry.addLine("Initializing Pedro shot trainer...");
        BjornHardware hardware = BjornHardware.forAutonomous(hardwareMap);
        shooter = new ShooterController(hardware.wheel, hardware.wheel2, hardware.intake, hardware.lift);

        // Initialize battery sensor like JULESVirtualShooter – first available pack sensor
        voltageSensor = firstVoltageSensor();

        try {
            drive = Constants.createFollower(hardwareMap);
        } catch (Exception e) {
            drive = null;
            RobotLog.ee(TAG, "Pedro follower init failed: %s", e.getMessage());
        }

        botId = ShotTrainerSettings.getBotId(hardwareMap.appContext);
        if (botId == null || botId.isEmpty()) {
            botId = "shot-trainer";
        }
        bridgeManager = JulesBridgeManager.getInstance();
        if (bridgeManager != null) {
            bridgeManager.prepare(hardwareMap.appContext);
            streamBus = bridgeManager.getStreamBus();
        }
        startBusListener();

        Pose startPose = poseForRange(START_RANGE_IN);
        if (drive != null) {
            try {
                drive.setStartingPose(startPose);
            } catch (Exception ignored) {
            }
        }
        pendingTargetPose = startPose;
        state = TrainState.INIT_POSITION;
        stateSinceMs = System.currentTimeMillis();
        telemetry.addData("session", sessionId);
        telemetry.addLine("Shot trainer ready. Press START.");
        telemetry.update();
    }

    @Override
    public void start() {
        sessionId = newSessionId();
        shotSeq = 0L;
        currentRangeIn = START_RANGE_IN;
        rangeDirection = +1;
        currentTargetRpm = 0.0;
        currentBaselineRpm = 0.0;
        currentOverrideRpm = Double.NaN;
        lastShotMetrics = null;
        lastShotTimestampMs = 0L;
        state = TrainState.INIT_POSITION;
        stateSinceMs = System.currentTimeMillis();
        goToRange(currentRangeIn);
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        if (streamBus == null && bridgeManager != null) {
            streamBus = bridgeManager.getStreamBus();
            if (streamBus != null) {
                startBusListener();
            }
        }
        if (drive != null) {
            drive.update();
        }
        shooter.update(nowMs);
        handleIncomingCommands(nowMs);

        // Use the real pack sensor when present
        currentBatteryV = readVoltage();

        Pose pose = drive != null ? drive.getPose() : null;
        double measuredRange = measuredRangeIn(pose);
        switch (state) {
            case INIT_POSITION:
            case DRIVING_TO_RANGE:
                if (isAtTargetRange()) {
                    updateTargetRpmForRange(currentRangeIn, nowMs);
                    transitionTo(TrainState.SPINNING_UP, nowMs);
                } else {
                    transitionTo(TrainState.DRIVING_TO_RANGE, nowMs);
                }
                break;

            case SPINNING_UP:
                if (shooter.isReady(nowMs)) {
                    transitionTo(TrainState.READY_TO_FIRE, nowMs);
                }
                break;

            case READY_TO_FIRE:
                if (shooter.fire(nowMs)) {
                    publishCmdStatus("fire", "accepted", null);
                    transitionTo(TrainState.FIRING, nowMs);
                } else {
                    publishCmdStatus("fire", "rejected", null);
                }
                break;

            case FIRING: {
                ShooterController.ShotMetrics metrics = shooter.pollShotMetrics();
                if (metrics != null) {
                    lastShotMetrics = metrics;
                    lastShotTimestampMs = nowMs;
                    publishShotEvent(metrics);
                    transitionTo(TrainState.WAITING_REWARD, nowMs);
                }
                break;
            }

            case WAITING_REWARD:
                if (nowMs - stateSinceMs >= WAIT_FOR_REWARD_MS) {
                    shooter.stop(nowMs);
                    currentTargetRpm = 0.0;
                    transitionTo(TrainState.ADVANCING_RANGE, nowMs);
                }
                break;

            case ADVANCING_RANGE:
                goToRange(nextRangeIn());
                transitionTo(TrainState.DRIVING_TO_RANGE, nowMs);
                break;

            case DONE:
            default:
                shooter.stop(nowMs);
                break;
        }

        updateTelemetry(pose, nowMs, measuredRange);
    }

    @Override
    public void stop() {
        shooter.stop(System.currentTimeMillis());
        if (busSub != null) {
            try {
                busSub.close();
            } catch (Exception ignored) {
            }
            busSub = null;
        }
        if (busPump != null) {
            busPump.interrupt();
            busPump = null;
        }
        pendingCmds.clear();
    }

    private void startBusListener() {
        if (streamBus == null || busSub != null) {
            return;
        }
        try {
            busSub = streamBus.subscribe();
            busPump = new Thread(() -> {
                try {
                    while (!Thread.currentThread().isInterrupted()) {
                        String line = busSub.take();
                        if (line == null) {
                            break;
                        }
                        routeBusLine(line);
                    }
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                } catch (Throwable ignored) {
                }
            }, "ShotTrainerBus");
            busPump.setDaemon(true);
            busPump.start();
        } catch (Throwable ignored) {
        }
    }

    private void handleIncomingCommands(long nowMs) {
        String raw;
        while ((raw = pendingCmds.poll()) != null) {
            try {
                JsonElement parsed = GsonCompat.parse(raw);
                if (parsed != null && parsed.isJsonObject()) {
                    JsonObject obj = parsed.getAsJsonObject();
                    String cmd = optString(obj, "cmd");
                    if (cmd != null) {
                        handleStructuredCommand(cmd.toLowerCase(Locale.US), obj, nowMs);
                    }
                }
            } catch (Throwable ignored) {
            }
        }
    }

    private boolean handleStructuredCommand(String cmd, JsonObject payload, long nowMs) {
        switch (cmd) {
            case "set_rpm": {
                double rpm = optDouble(payload, new String[]{"rpm", "value", "target"}, currentTargetRpm);
                currentOverrideRpm = Range.clip(rpm, RPM_MIN, RPM_MAX);
                publishCmdStatus("set_rpm", "accepted", payload);
                if (rpmControlMode == RpmControlMode.LEARNED) {
                    updateTargetRpmForRange(currentRangeIn, nowMs);
                }
                return true;
            }
            case "clear_rpm_override": {
                currentOverrideRpm = Double.NaN;
                publishCmdStatus("clear_rpm_override", "accepted", payload);
                if (rpmControlMode == RpmControlMode.LEARNED) {
                    updateTargetRpmForRange(currentRangeIn, nowMs);
                }
                return true;
            }
            case "set_rpm_mode": {
                String mode = optString(payload, "mode");
                if (mode == null) {
                    mode = optString(payload, "value");
                }
                if (mode == null) {
                    publishCmdStatus("set_rpm_mode", "rejected", payload);
                    return true;
                }
                RpmControlMode previous = rpmControlMode;
                rpmControlMode = "learned".equalsIgnoreCase(mode)
                        ? RpmControlMode.LEARNED
                        : RpmControlMode.BASELINE;
                publishCmdStatus("set_rpm_mode", "accepted", payload);
                if (previous != rpmControlMode) {
                    updateTargetRpmForRange(currentRangeIn, nowMs);
                }
                return true;
            }
            default:
                return false;
        }
    }

    private void routeBusLine(String line) {
        if (line == null || line.isEmpty()) {
            return;
        }
        try {
            JsonElement el = GsonCompat.parse(line);
            if (el == null || !el.isJsonObject()) {
                return;
            }
            JsonObject obj = el.getAsJsonObject();
            String type = optString(obj, "type");
            if ("cmd".equalsIgnoreCase(type)) {
                JsonElement text = obj.get("text");
                if (text != null) {
                    if (text.isJsonObject()) {
                        pendingCmds.offer(text.getAsJsonObject().toString());
                    } else if (text.isJsonPrimitive()) {
                        pendingCmds.offer(text.getAsString());
                    }
                }
                return;
            }
            if (obj.has("cmd")) {
                pendingCmds.offer(obj.toString());
            }
        } catch (Throwable ignored) {
        }
    }

    private static String optString(JsonObject obj, String field) {
        if (obj == null || field == null) {
            return null;
        }
        JsonElement el = obj.get(field);
        if (el == null) {
            return null;
        }
        try {
            return el.getAsString();
        } catch (Exception e) {
            return null;
        }
    }

    private static double optDouble(JsonObject obj, String[] keys, double def) {
        if (obj == null || keys == null) {
            return def;
        }
        for (String key : keys) {
            if (key == null) {
                continue;
            }
            double value = optDouble(obj, key, Double.NaN);
            if (!Double.isNaN(value)) {
                return value;
            }
        }
        return def;
    }

    private static double optDouble(JsonObject obj, String key, double def) {
        if (obj == null || key == null) {
            return def;
        }
        JsonElement el = obj.get(key);
        return optDouble(el, def);
    }

    private static double optDouble(JsonElement el, double def) {
        if (el == null) {
            return def;
        }
        try {
            return el.getAsDouble();
        } catch (Exception ignored) {
            if (el.isJsonPrimitive()) {
                try {
                    return Double.parseDouble(el.getAsString());
                } catch (Exception ignored2) {
                    return def;
                }
            }
            return def;
        }
    }

    private void goToRange(double rangeIn) {
        currentRangeIn = clampRange(rangeIn);
        pendingTargetPose = poseForRange(currentRangeIn);
        if (drive == null || pendingTargetPose == null) {
            return;
        }
        Pose start = drive.getPose();
        if (start == null) {
            start = pendingTargetPose;
        }
        try {
            PathChain path = drive.pathBuilder()
                    .addPath(new BezierLine(start, pendingTargetPose))
                    .setLinearHeadingInterpolation(start.getHeading(), pendingTargetPose.getHeading())
                    .build();
            drive.followPath(path, true);
        } catch (Exception e) {
            RobotLog.ee(TAG, "Failed to build Pedro path: %s", e.getMessage());
        }
    }

    private Pose poseForRange(double rangeIn) {
        // March along the -X axis: basePoseNearGoal.x is "0 inches from goal"
        // and increasing range moves the robot to more negative X.
        double targetX = basePoseNearGoal.getX() - rangeIn;
        double targetY = basePoseNearGoal.getY();
        return new Pose(targetX, targetY, TRAIN_HEADING_RAD);
    }

    private double nextRangeIn() {
        double next = currentRangeIn + rangeDirection * STEP_IN;
        if (next > MAX_RANGE_IN) {
            next = MAX_RANGE_IN;
            rangeDirection = -1;
        } else if (next < MIN_RANGE_IN) {
            next = MIN_RANGE_IN;
            rangeDirection = +1;
        }
        currentRangeIn = next;
        return currentRangeIn;
    }

    private void transitionTo(TrainState newState, long nowMs) {
        if (state != newState) {
            state = newState;
            stateSinceMs = nowMs;
        }
    }

    private void updateTargetRpmForRange(double rangeIn, long nowMs) {
        currentBaselineRpm = lookupRpmForRange(rangeIn);
        double effectiveRpm = currentBaselineRpm;
        if (rpmControlMode == RpmControlMode.LEARNED && !Double.isNaN(currentOverrideRpm)) {
            effectiveRpm = blendRpm(currentBaselineRpm, currentOverrideRpm);
        }
        currentTargetRpm = effectiveRpm;
        shooter.setTargetRpm(currentTargetRpm, nowMs);
    }

    private double blendRpm(double baseline, double override) {
        double blended = 0.5 * baseline + 0.5 * override;
        double delta = blended - baseline;
        if (Math.abs(delta) > MAX_OVERRIDE_DELTA_RPM) {
            blended = baseline + Math.copySign(MAX_OVERRIDE_DELTA_RPM, delta);
        }
        return Range.clip(blended, RPM_MIN, RPM_MAX);
    }

    private boolean isAtTargetRange() {
        if (drive == null) {
            return true;
        }
        Pose pose = drive.getPose();
        if (pose == null) {
            return false;
        }
        double rangeError = Math.abs(measuredRangeIn(pose) - currentRangeIn);
        double headingError = Math.abs(angleWrap(pose.getHeading() - TRAIN_HEADING_RAD));
        boolean settled = !drive.isBusy();
        return rangeError <= RANGE_TOLERANCE_IN && headingError <= HEADING_TOLERANCE_RAD && settled;
    }

    /**
     * Baseline hand-tuned model from distance -> RPM.
     * Part A: always use this.
     * Part B: becomes the fallback / blend partner for RL suggestions.
     */
    private double lookupRpmForRange(double rangeIn) {
        double linear = 2100.0 + (rangeIn * 8.0);
        return Range.clip(linear, RPM_MIN, RPM_MAX);
    }

    private double clampRange(double rangeIn) {
        return Math.max(MIN_RANGE_IN, Math.min(MAX_RANGE_IN, rangeIn));
    }

    private double measuredRangeIn() {
        Pose pose = drive != null ? drive.getPose() : null;
        return measuredRangeIn(pose);
    }

    private double measuredRangeIn(Pose pose) {
        if (pose == null) {
            return currentRangeIn;
        }
        // Range is "how far we moved in -X from the base pose"
        return basePoseNearGoal.getX() - pose.getX();
    }

    private double angleWrap(double angleRad) {
        double wrapped = angleRad;
        while (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        while (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }

    /**
     * Match the virtual shooter pattern: grab the first available VoltageSensor
     * from the hardware map, without going through BjornHardware.
     */
    private VoltageSensor firstVoltageSensor() {
        try {
            for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
                return vs;
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    /**
     * Read pack voltage safely. Lazily re-resolves the sensor if needed and
     * falls back to the last-known simulated value if anything goes wrong.
     */
    private double readVoltage() {
        // Lazy-init / re-acquire sensor if it wasn't ready in init()
        if (voltageSensor == null) {
            voltageSensor = firstVoltageSensor();
        }
        if (voltageSensor != null) {
            try {
                double v = voltageSensor.getVoltage();
                if (Double.isFinite(v) && v > 0) {
                    return v;
                }
            } catch (Exception ignored) {
            }
        }
        // Fallback to currentBatteryV (e.g. sim/default)
        return currentBatteryV;
    }

    private void updateTelemetry(Pose pose, long nowMs, double measuredRange) {
        telemetry.addData("state", state);
        telemetry.addData("session", sessionId);
        telemetry.addData("bot", botId);
        telemetry.addData("range_in", String.format(Locale.US, "%.1f", currentRangeIn));
        telemetry.addData("range_measured_in", String.format(Locale.US, "%.1f", measuredRange));
        telemetry.addData("range_dir", rangeDirection > 0 ? "away" : "toward");
        telemetry.addData("rpm_mode", rpmControlMode);
        telemetry.addData("target_rpm", String.format(Locale.US, "%.0f", currentTargetRpm));
        telemetry.addData("baseline_rpm", String.format(Locale.US, "%.0f", currentBaselineRpm));
        telemetry.addData("override_rpm",
                Double.isNaN(currentOverrideRpm) ? "n/a" : String.format(Locale.US, "%.0f", currentOverrideRpm));
        telemetry.addData("battery_v", String.format(Locale.US, "%.2f", currentBatteryV));
        telemetry.addData("shooter_ready", shooter.isReady(nowMs));
        if (pose != null) {
            telemetry.addData("pose",
                    String.format(Locale.US, "x=%.1f y=%.1f h=%.1f",
                            pose.getX(),
                            pose.getY(),
                            Math.toDegrees(pose.getHeading())));
        }
        if (lastShotMetrics != null) {
            telemetry.addData("last_shot_rpm", lastShotMetrics.rpmAtFire);
            telemetry.addData("last_ready_latency_ms", lastShotMetrics.timeToReadyMs);
            telemetry.addData("last_shot_ts", lastShotTimestampMs);
        }
        telemetry.update();
    }

    private String newSessionId() {
        return String.format(Locale.US, "shot-%08x", System.currentTimeMillis() & 0xffffffffL);
    }

    private void publishShotEvent(ShooterController.ShotMetrics metrics) {
        JsonObject ev = new JsonObject();
        ev.addProperty("type", "shot");
        ev.addProperty("ts_ms", System.currentTimeMillis());
        long seq = ++shotSeq;
        ev.addProperty("shot_id", sessionId + "-" + seq);
        ev.addProperty("session_id", sessionId);
        ev.addProperty("bot_id", botId);
        ev.addProperty("battery_v", currentBatteryV);
        double measuredRange = measuredRangeIn();
        ev.addProperty("distance_in", measuredRange);
        ev.addProperty("rpm_at_fire", metrics.rpmAtFire);
        ev.addProperty("ready_latency_ms", metrics.timeToReadyMs);
        ev.addProperty("fire_command_ms", metrics.fireTimestampMs);

        JsonObject ctx = new JsonObject();
        ctx.addProperty("target_rpm", currentTargetRpm);
        ctx.addProperty("manual_target_rpm", currentTargetRpm);
        ctx.addProperty("range_in", measuredRange);
        ctx.addProperty("range_target_in", currentRangeIn);
        ctx.addProperty("pose_heading_rad", TRAIN_HEADING_RAD);
        ctx.addProperty("rpm_mode", rpmControlMode.name().toLowerCase(Locale.US));
        ctx.addProperty("baseline_rpm", currentBaselineRpm);
        if (!Double.isNaN(currentOverrideRpm)) {
            ctx.addProperty("override_rpm", currentOverrideRpm);
        }
        ev.add("context", ctx);
        busPublish(ev.toString());
    }

    private void publishCmdStatus(String name, String status, JsonObject args) {
        JsonObject ev = new JsonObject();
        ev.addProperty("type", "cmd_status");
        ev.addProperty("name", name);
        ev.addProperty("status", status);
        ev.addProperty("ts_ms", System.currentTimeMillis());
        if (args != null) {
            ev.add("args", args);
        }
        busPublish(ev.toString());
    }

    private void busPublish(String json) {
        if (json == null || json.isEmpty()) {
            return;
        }
        JulesStreamBus bus = resolveStreamBus();
        if (bus == null) {
            return;
        }
        try {
            bus.publishJsonLine(json);
        } catch (Throwable ignored) {
        }
    }

    private JulesStreamBus resolveStreamBus() {
        if (bridgeManager != null) {
            JulesStreamBus shared = bridgeManager.getStreamBus();
            if (shared != null) {
                streamBus = shared;
            }
        }
        return streamBus;
    }
}
