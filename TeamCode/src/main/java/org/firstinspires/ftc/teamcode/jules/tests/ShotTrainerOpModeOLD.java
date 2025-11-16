package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonObject;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.jules.shot.ShotTrainerSettings;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Iterator;
import java.util.Locale;

/**
 * Pedro-driven shot trainer that walks the robot through a set of ranges and reports shots on the
 * primary JULES telemetry stream so the desktop RL client can score them.
 */
@Autonomous(name = "JULES: Shot TrainerO OLD", group = "JULES")
public final class ShotTrainerOpModeOLD extends OpMode {

    private static final String TAG = "ShotTrainerOpMode";

    // Distance band for training.
    private static final double MIN_RANGE_IN = 10.0;
    private static final double MAX_RANGE_IN = 64.0;
    private static final double STEP_IN = 4.0;
    private static final double START_RANGE_IN = 25.0;

    // Pedro tolerances for "at range".
    private static final double RANGE_TOLERANCE_IN = 0.5;
    private static final double HEADING_TOLERANCE_RAD = Math.toRadians(3.0);

    // 0" = closest legal shooting line. Positive = away from goal.
    private static final double TRAIN_HEADING_RAD = Math.toRadians(90.0);

    private static final long WAIT_FOR_REWARD_MS = 5_000L;

    private VoltageSensor voltageSensor;
    private ShooterController shooter;
    private Follower drive;
    private Pose basePoseNearGoal = new Pose(0.0, 0.0, TRAIN_HEADING_RAD);
    private Pose pendingTargetPose;

    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;

    private double currentRangeIn = START_RANGE_IN;
    private int rangeDirection = +1;
    private double currentTargetRpm = 0.0;
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

    @Override
    public void init() {
        telemetry.addLine("Initializing Pedro shot trainer...");
        BjornHardware hardware = BjornHardware.forAutonomous(hardwareMap);
        shooter = new ShooterController(hardware.wheel, hardware.wheel2, hardware.intake, hardware.lift);
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
        lastShotMetrics = null;
        lastShotTimestampMs = 0L;
        state = TrainState.INIT_POSITION;
        stateSinceMs = System.currentTimeMillis();
        goToRange(currentRangeIn);
    }

    @Override
    public void loop() {
        long nowMs = System.currentTimeMillis();
        if (drive != null) {
            drive.update();
        }
        shooter.update(nowMs);
        currentBatteryV = readVoltage();

        Pose pose = drive != null ? drive.getPose() : null;
        double measuredRange = measuredRangeIn(pose);
        switch (state) {
            case INIT_POSITION:
            case DRIVING_TO_RANGE:
                if (isAtTargetRange()) {
                    currentTargetRpm = lookupRpmForRange(currentRangeIn);
                    shooter.setTargetRpm(currentTargetRpm, nowMs);
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
        double targetX = basePoseNearGoal.getX() + rangeIn;
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

    private boolean isAtTargetRange() {
        if (drive == null) {
            return true;
        }
        Pose pose = drive.getPose();
        Pose target = pendingTargetPose != null ? pendingTargetPose : poseForRange(currentRangeIn);
        if (pose == null || target == null) {
            return false;
        }
        double dx = target.getX() - pose.getX();
        double dy = target.getY() - pose.getY();
        double posError = Math.hypot(dx, dy);
        double headingError = Math.abs(angleWrap(pose.getHeading() - TRAIN_HEADING_RAD));
        boolean settled = !drive.isBusy();
        return posError <= RANGE_TOLERANCE_IN && headingError <= HEADING_TOLERANCE_RAD && settled;
    }

    private double lookupRpmForRange(double rangeIn) {
        // TODO: Replace with a learned model or interpolation map provided by the client.
        double linear = 2100.0 + (rangeIn * 8.0);
        return Math.max(1800.0, Math.min(3000.0, linear));
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
        return pose.getX() - basePoseNearGoal.getX();
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

    private VoltageSensor firstVoltageSensor() {
        Iterator<VoltageSensor> it = hardwareMap.voltageSensor.iterator();
        return it.hasNext() ? it.next() : null;
    }

    private double readVoltage() {
        if (voltageSensor == null) {
            return currentBatteryV;
        }
        try {
            double v = voltageSensor.getVoltage();
            if (Double.isFinite(v) && v > 0) {
                return v;
            }
        } catch (Exception ignored) {
        }
        return currentBatteryV;
    }

    private void updateTelemetry(Pose pose, long nowMs, double measuredRange) {
        telemetry.addData("state", state);
        telemetry.addData("session", sessionId);
        telemetry.addData("bot", botId);
        telemetry.addData("range_in", String.format(Locale.US, "%.1f", currentRangeIn));
        telemetry.addData("range_measured_in", String.format(Locale.US, "%.1f", measuredRange));
        telemetry.addData("range_dir", rangeDirection > 0 ? "away" : "toward");
        telemetry.addData("target_rpm", String.format(Locale.US, "%.0f", currentTargetRpm));
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
