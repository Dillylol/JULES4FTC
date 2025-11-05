package org.firstinspires.ftc.teamcode.jules.tests;

import static java.lang.Math.abs;

import androidx.annotation.Nullable;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.shot.MailboxConsumer;
import org.firstinspires.ftc.teamcode.jules.shot.PedroShotNavigator;
import org.firstinspires.ftc.teamcode.jules.shot.RpmProvider;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.jules.shot.ShotPlan;
import org.firstinspires.ftc.teamcode.jules.shot.ShotPlannerBridge;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Integrated trainer OpMode that executes the request → spin → fire loop against the planner.
 */
@Autonomous(name = "JULES: Shot Planner", group = "JULES")
public final class ShotTrainerOpMode extends LinearOpMode {

    private static final String TAG = "ShotTrainerOpMode";

    private static final double MIN_RANGE_IN = 28.0;
    private static final double MAX_RANGE_IN = 68.0;
    private static final double INITIAL_RANGE_IN = 36.0;
    private static final double AIM_TOLERANCE_DEG = 6.0;
    private static final double RANGE_TOL_IN = 0.75;
    private static final long OBS_INTERVAL_MS = 80L;
    private static final long PLAN_REQUEST_INTERVAL_MS = 200L;
    private static final int WARMUP_SHOTS = 2;
    private static final double BATTERY_ABORT_V = 11.0;

    private VoltageSensor voltageSensor;
    private ShooterController shooter;
    private PedroShotNavigator navigator;
    private RpmProvider rpmProvider;
    private ShotPlannerBridge bridge;
    private MailboxConsumer mailbox;

    private boolean warmupComplete = false;
    private int warmupShotsRemaining = WARMUP_SHOTS;
    private final List<Double> warmupBiasSamples = new ArrayList<>();

    private ShotPlan activePlan;
    private boolean planShotTriggered = false;
    private boolean waitingForCommand = true;
    private double currentHeadingOffsetDeg = 0.0;

    private long lastObsSentMs = 0L;
    private long lastPlanRequestMs = -PLAN_REQUEST_INTERVAL_MS;
    private long sessionStartMs = 0L;

    private boolean sessionAborted = false;

    @Override
    public void runOpMode() {
        BjornHardware hardware = BjornHardware.forAutonomous(hardwareMap);
        DcMotorEx wheel = hardware.wheel;
        DcMotorEx intake = hardware.intake;
        Servo lift = hardware.lift;
        voltageSensor = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;

        Follower follower = Constants.createFollower(hardwareMap);
        Pose tagPose = new Pose(0, 0, Math.toRadians(0));
        try {
            double startX = tagPose.getX() - Math.cos(tagPose.getHeading()) * INITIAL_RANGE_IN;
            double startY = tagPose.getY() - Math.sin(tagPose.getHeading()) * INITIAL_RANGE_IN;
            follower.setStartingPose(new Pose(startX, startY, tagPose.getHeading()));
        } catch (Exception ignored) {
        }

        navigator = new PedroShotNavigator(
                follower,
                tagPose,
                AIM_TOLERANCE_DEG,
                RANGE_TOL_IN,
                INITIAL_RANGE_IN,
                MIN_RANGE_IN,
                MAX_RANGE_IN);

        shooter = new ShooterController(wheel, intake, lift);
        rpmProvider = new RpmProvider();

        JulesBridgeManager bridgeManager = JulesBridgeManager.getInstance();
        bridgeManager.prepare(hardwareMap.appContext);
        JulesStreamBus bus = bridgeManager.getStreamBus();
        bridge = new ShotPlannerBridge(bus);
        mailbox = new MailboxConsumer(bridge, rpmProvider);

        loadInitialModel();

        telemetry.addLine("JULES Shot Planner ready. Press START.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        sessionStartMs = 0L;
        String sessionId = String.format(Locale.US, "trainer-%d", System.currentTimeMillis());
        bridge.setSessionId(sessionId);

        while (opModeIsActive() && !isStopRequested() && !sessionAborted) {
            long nowMs = (long) runtime.milliseconds();
            if (sessionStartMs == 0L) {
                sessionStartMs = nowMs;
                bridge.sendHello(sessionId, nowMs);
            }

            boolean manualOverride = isManualOverride();

            navigator.update();
            navigator.driveToTarget();

            double rawVoltage = readVoltage();
            double filteredVoltage = rpmProvider.updateAndGetLoadVoltage(nowMs, rawVoltage, shooter.isUnderLoad());
            double rangeIn = navigator.getRangeIn();
            double headingToTag = navigator.getHeadingToTagDeg();

            RpmProvider.Target target = rpmProvider.target(rangeIn, filteredVoltage);
            double rpmBase = target.rpmBase;
            double rpmCommand = target.rpmTarget;

            if (warmupComplete) {
                ShotPlan planUpdate = mailbox.poll(nowMs);
                if (planUpdate != null) {
                    handlePlan(planUpdate, nowMs);
                }

                if (activePlan != null && nowMs > activePlan.validUntilMs) {
                    RobotLog.w(TAG, "Command %s expired before execution", activePlan.cmdId);
                    activePlan = null;
                    planShotTriggered = false;
                    waitingForCommand = true;
                }

                if (activePlan != null) {
                    if (activePlan.hasAbsoluteTarget()) {
                        Double absolute = activePlan.getRpmTargetAbs();
                        if (absolute != null) {
                            rpmCommand = absolute;
                        }
                    } else {
                        Double bias = activePlan.getRpmBias();
                        if (bias != null) {
                            rpmCommand = target.rpmTarget + bias;
                        }
                    }
                }
            }

            if (manualOverride || sessionAborted) {
                shooter.stop(nowMs);
            } else {
                shooter.setTargetRpm(rpmCommand, nowMs);
            }

            shooter.update(nowMs);

            ShotPlannerBridge.PoseSnapshot pose = navigator.getPoseSnapshot();
            ShotPlannerBridge.ShotContext context = new ShotPlannerBridge.ShotContext(
                    pose,
                    rangeIn,
                    filteredVoltage,
                    shooter.getMeasuredRpm(),
                    rpmBase,
                    rpmCommand,
                    headingToTag);

            if (nowMs - lastObsSentMs >= OBS_INTERVAL_MS) {
                bridge.sendObservation(context, nowMs);
                lastObsSentMs = nowMs;
            }

            if (!warmupComplete) {
                runWarmup(nowMs, context, rpmBase, rpmCommand, pose, headingToTag);
            } else {
                executePlanLoop(nowMs, manualOverride, context, rpmCommand, pose, headingToTag);
            }

            updateTelemetry(nowMs, manualOverride, filteredVoltage, rpmBase, rpmCommand);
            idle();
        }

        shooter.stop((long) runtime.milliseconds());
    }

    private void runWarmup(long nowMs,
                           ShotPlannerBridge.ShotContext context,
                           double rpmBase,
                           double rpmCommand,
                           ShotPlannerBridge.PoseSnapshot pose,
                           double headingToTag) {
        if (warmupShotsRemaining <= 0) {
            finalizeWarmup();
            waitingForCommand = true;
            return;
        }

        boolean aimed = navigator.isAimed();
        boolean ready = shooter.isReady(nowMs);

        if (aimed && ready && !shooter.isLockedOut(nowMs)) {
            shooter.fire(nowMs);
        }

        ShooterController.ShotMetrics metrics = shooter.pollShotMetrics();
        if (metrics != null) {
            warmupShotsRemaining -= 1;
            double biasSample = metrics.rpmAtFire - rpmBase;
            warmupBiasSamples.add(biasSample);
            String trialId = String.format(Locale.US, "warmup_%d", WARMUP_SHOTS - warmupShotsRemaining);
            ShotPlannerBridge.ShotToken token = new ShotPlannerBridge.ShotToken(
                    trialId,
                    metrics.fireTimestampMs,
                    context.rangeIn,
                    context.vBattLoad,
                    rpmCommand,
                    metrics.rpmAtFire,
                    metrics.timeToReadyMs,
                    pose,
                    headingToTag);
            bridge.sendShotToken(token);
        }

        if (warmupShotsRemaining <= 0) {
            finalizeWarmup();
            waitingForCommand = true;
        }
    }

    private void finalizeWarmup() {
        warmupComplete = true;
        double biasSum = 0.0;
        for (Double sample : warmupBiasSamples) {
            biasSum += sample;
        }
        double sessionBias = warmupBiasSamples.isEmpty() ? 0.0 : biasSum / warmupBiasSamples.size();
        rpmProvider.setSessionBias(sessionBias);
        RobotLog.ii(TAG, "Warmup complete; session bias %.2f", sessionBias);
    }

    private void executePlanLoop(long nowMs,
                                 boolean manualOverride,
                                 ShotPlannerBridge.ShotContext context,
                                 double rpmCommand,
                                 ShotPlannerBridge.PoseSnapshot pose,
                                 double headingToTag) {
        if (sessionAborted) {
            return;
        }

        if (context.vBattLoad < BATTERY_ABORT_V) {
            sessionAborted = true;
            RobotLog.e(TAG, "Battery below floor %.2fV; aborting", context.vBattLoad);
            shooter.stop(nowMs);
            return;
        }

        if (waitingForCommand && nowMs - lastPlanRequestMs >= PLAN_REQUEST_INTERVAL_MS) {
            bridge.sendRequestShotPlan(context, nowMs);
            lastPlanRequestMs = nowMs;
        }

        if (activePlan != null && !manualOverride) {
            boolean aimed = navigator.isAimed();
            boolean ready = shooter.isReady(nowMs);
            boolean loiter = activePlan.loiter;

            if (!loiter && aimed && ready && !planShotTriggered && !shooter.isLockedOut(nowMs)) {
                if (shooter.fire(nowMs)) {
                    planShotTriggered = true;
                }
            }
        }

        ShooterController.ShotMetrics metrics = shooter.pollShotMetrics();
        if (metrics != null) {
            String trialId = (activePlan != null) ? activePlan.trialId : String.format(Locale.US, "shot_%d", metrics.fireTimestampMs);
            ShotPlannerBridge.ShotToken token = new ShotPlannerBridge.ShotToken(
                    trialId,
                    metrics.fireTimestampMs,
                    context.rangeIn,
                    context.vBattLoad,
                    rpmCommand,
                    metrics.rpmAtFire,
                    metrics.timeToReadyMs,
                    pose,
                    headingToTag);
            bridge.sendShotToken(token);
            planShotTriggered = false;
            if (activePlan != null && !activePlan.loiter) {
                waitingForCommand = true;
                activePlan = null;
            }
        }
    }

    private void handlePlan(ShotPlan plan, long nowMs) {
        if (warmupComplete) {
            navigator.applyRangeDelta(plan.rangeDeltaIn);
            navigator.setHeadingOffsetDeg(plan.headingOffsetDeg);
            currentHeadingOffsetDeg = plan.headingOffsetDeg;
            activePlan = plan;
            planShotTriggered = false;
            waitingForCommand = plan.loiter;
            if (waitingForCommand) {
                lastPlanRequestMs = nowMs;
            }
        }
    }

    private void loadInitialModel() {
        try {
            InputStream stream = hardwareMap.appContext.getAssets().open("rpm_model.json");
            try (BufferedReader reader = new BufferedReader(new InputStreamReader(stream, StandardCharsets.UTF_8))) {
                StringBuilder sb = new StringBuilder();
                String line;
                while ((line = reader.readLine()) != null) {
                    sb.append(line);
                }
                JsonElement parsed = GsonCompat.parse(sb.toString());
                if (parsed != null && parsed.isJsonObject()) {
                    JsonObject obj = parsed.getAsJsonObject();
                    rpmProvider.applyUpdate(obj);
                    RobotLog.i(TAG, "Loaded default rpm_model.json");
                }
            }
        } catch (Exception e) {
            RobotLog.w(TAG, "No default rpm_model.json found: %s", e.getMessage());
        }
    }

    private void updateTelemetry(long nowMs,
                                 boolean manualOverride,
                                 double voltage,
                                 double rpmBase,
                                 double rpmCmd) {
        telemetry.addData("state", warmupComplete ? (activePlan != null ? "plan" : (waitingForCommand ? "await_cmd" : "idle")) : "warmup");
        telemetry.addData("manual", manualOverride);
        telemetry.addData("rangeTarget", String.format(Locale.US, "%.1f", navigator.getRangeTarget()));
        telemetry.addData("rangeIn", String.format(Locale.US, "%.1f", navigator.getRangeIn()));
        telemetry.addData("headingOffset", String.format(Locale.US, "%.1f", currentHeadingOffsetDeg));
        telemetry.addData("batteryV", String.format(Locale.US, "%.2f", voltage));
        telemetry.addData("rpmBase", String.format(Locale.US, "%.0f", rpmBase));
        telemetry.addData("rpmCmd", String.format(Locale.US, "%.0f", rpmCmd));
        telemetry.addData("rpmMeas", String.format(Locale.US, "%.0f", shooter.getMeasuredRpm()));
        telemetry.addData("ready", shooter.isReady(nowMs));
        telemetry.addData("aimed", navigator.isAimed());
        telemetry.addData("planId", activePlan != null ? activePlan.cmdId : "none");
        telemetry.addData("loiter", activePlan != null && activePlan.loiter);
        telemetry.addData("sessionBias", String.format(Locale.US, "%.1f", rpmProvider.getSessionBias()));
        telemetry.update();
    }

    private boolean isManualOverride() {
        return abs(gamepad1.left_stick_x) > 0.15
                || abs(gamepad1.left_stick_y) > 0.15
                || abs(gamepad2.left_stick_x) > 0.15
                || abs(gamepad2.left_stick_y) > 0.15
                || gamepad1.right_trigger > 0.2
                || gamepad2.right_trigger > 0.2;
    }

    private double readVoltage() {
        if (voltageSensor == null) {
            return 12.0;
        }
        try {
            double v = voltageSensor.getVoltage();
            if (Double.isNaN(v) || v <= 0) {
                return 12.0;
            }
            return v;
        } catch (Exception e) {
            return 12.0;
        }
    }
}