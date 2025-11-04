package org.firstinspires.ftc.teamcode.jules.shot;

import com.google.gson.JsonObject;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;
import java.util.Locale;

/**
 * TeleOp module that executes the single-shot curriculum loop.
 */
@TeleOp(name = "JULES Shot Trainer", group = "Jules")
public final class ShotTrainerOpMode extends OpMode {
    private enum State { DRIVE_AIM, REQUEST_PLAN, SPINUP_WAIT, FIRE_SEQ, RESULT_WAIT }

    private static final double MIN_RANGE_IN = 28.0;
    private static final double MAX_RANGE_IN = 68.0;
    private static final double STEP_IN = 4.0;
    private static final int RPM_READY_TOL = 30;
    private static final int RPM_DIP_THRESHOLD = 120;
    private static final long POST_FIRE_WINDOW_MS = 1500L;
    private static final double TAG_X_IN = 0.0;
    private static final double TAG_Y_IN = 0.0;
    private static final double APPROACH_AXIS_DEG = 0.0; // robot downfield of tag
    private static final double POSE_TOL_IN = 1.0;
    private static final double HEADING_TOL_DEG = 2.0;
    private static final double GATE_TRIGGER_IN = 5.0;

    private final ElapsedTime clock = new ElapsedTime();

    private Follower follower;
    private PedroShotNavigator navigator;
    private ShooterController shooter;
    private DistanceSensor gateSensor;
    private List<VoltageSensor> voltageSensors;

    private RpmProvider rpmProvider;
    private ShotPlannerBridge plannerBridge;
    private MailboxConsumer mailboxConsumer;

    private ShotPlan currentPlan = ShotPlan.idle();
    private ShotPlan pendingPlan;

    private double rangeTargetIn = MIN_RANGE_IN;
    private double lastBattery = 12.0;
    private double lastRange = MIN_RANGE_IN;
    private double lastHeadingToTag = 0.0;
    private double rpmCommanded = 0.0;
    private double rpmMeasured = 0.0;
    private double rpmHint = 0.0;

    private State state = State.DRIVE_AIM;
    private boolean planRequested;
    private boolean fireNowLatched;
    private long fireCommandedAtMs = -1;

    private long resultWindowDeadlineMs;
    private boolean shotFired;
    private boolean observationSent;
    private int rpmAtFire;
    private long shotDetectedMs;
    private int hitLatched = -1;
    private boolean gateTriggered;

    private boolean fireSeqPrimed;
    private boolean resultPrimed;

    private boolean g1APrev, g1BPrev, g2APrev, g2BPrev;

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(75);
        follower = Constants.createFollower(hardwareMap);
        navigator = new PedroShotNavigator(follower, TAG_X_IN, TAG_Y_IN, APPROACH_AXIS_DEG, POSE_TOL_IN, HEADING_TOL_DEG);
        navigator.initialize(rangeTargetIn);
        Pose start = new Pose(TAG_X_IN + rangeTargetIn, TAG_Y_IN, Math.toRadians(APPROACH_AXIS_DEG + 180.0));
        try { follower.setStartingPose(start); } catch (Exception ignored) { }

        BjornHardware hardware = BjornHardware.forAutonomous(hardwareMap);
        shooter = new ShooterController(hardware.wheel, hardware.intake, hardware.lift);
        gateSensor = hardware.frontTof;
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);

        JulesBridgeManager bridgeManager = JulesBridgeManager.getInstance();
        bridgeManager.prepare(hardwareMap.appContext);
        JulesStreamBus streamBus = bridgeManager.getStreamBus();

        rpmProvider = RpmProvider.withDefaults();
        plannerBridge = new ShotPlannerBridge(streamBus);
        mailboxConsumer = new MailboxConsumer(plannerBridge);

        clock.reset();
    }

    @Override
    public void start() {
        clock.reset();
    }

    @Override
    public void loop() {
        long nowMs = (long) clock.milliseconds();

        mailboxConsumer.poll(nowMs, new MailboxConsumer.Handler() {
            @Override
            public void onShotPlan(ShotPlan plan) {
                pendingPlan = plan;
            }

            @Override
            public void onRpmModelUpdate(JsonObject update) {
                rpmProvider.applyUpdate(update);
            }
        });

        if (pendingPlan != null) {
            currentPlan = pendingPlan;
            pendingPlan = null;
        }

        navigator.update();
        shooter.update(nowMs);

        PedroShotNavigator.PoseSnapshot pose = navigator.snapshot();
        lastRange = navigator.rangeToTag();
        lastHeadingToTag = navigator.headingToTagDeg();
        lastBattery = readBatteryVoltage();

        rpmHint = rpmProvider.targetRpm(lastRange, lastBattery);
        double bias = currentPlan.getRpmBias();
        rpmCommanded = Math.max(0.0, rpmHint + bias);
        rpmMeasured = shooter.getMeasuredRpm();

        int rpmTarget = (state == State.SPINUP_WAIT || state == State.FIRE_SEQ || state == State.RESULT_WAIT)
                ? (int) Math.round(rpmCommanded)
                : 0;
        shooter.setFlywheelTarget(rpmTarget);

        switch (state) {
            case DRIVE_AIM:
                navigator.driveToRangeAndAim(rangeTargetIn);
                shooter.closeLift();
                if (navigator.atAimPose(rangeTargetIn)) {
                    transitionTo(State.REQUEST_PLAN);
                }
                break;

            case REQUEST_PLAN:
                if (!planRequested) {
                    plannerBridge.sendRequestShotPlan(lastRange, lastBattery, pose, rpmHint);
                    planRequested = true;
                }
                transitionTo(State.SPINUP_WAIT);
                break;

            case SPINUP_WAIT:
                if (currentPlan.isFireNow()) {
                    fireNowLatched = true;
                    if (fireCommandedAtMs < 0) {
                        fireCommandedAtMs = currentPlan.getReceivedAtMs();
                    }
                    currentPlan.consumeFireNow();
                }
                boolean ready = shooter.isReady(RPM_READY_TOL);
                if (ready && fireNowLatched) {
                    transitionTo(State.FIRE_SEQ);
                }
                break;

            case FIRE_SEQ:
                beginFireSequence(nowMs);
                transitionTo(State.RESULT_WAIT);
                break;

            case RESULT_WAIT:
                if (!resultPrimed) {
                    resultPrimed = true;
                    resultWindowDeadlineMs = nowMs + POST_FIRE_WINDOW_MS;
                    observationSent = false;
                    shotFired = false;
                    shotDetectedMs = -1;
                    rpmAtFire = 0;
                    hitLatched = -1;
                    gateTriggered = false;
                    g1APrev = gamepad1 != null && gamepad1.a;
                    g1BPrev = gamepad1 != null && gamepad1.b;
                    g2APrev = gamepad2 != null && gamepad2.a;
                    g2BPrev = gamepad2 != null && gamepad2.b;
                }
                monitorShot(nowMs, pose);
                break;
        }

        renderTelemetry(pose);
    }

    private void monitorShot(long nowMs, PedroShotNavigator.PoseSnapshot pose) {
        if (!shotFired) {
            double drop = rpmCommanded - rpmMeasured;
            if (rpmCommanded > 0 && drop >= RPM_DIP_THRESHOLD) {
                shotFired = true;
                rpmAtFire = (int) Math.round(rpmMeasured);
                shotDetectedMs = nowMs;
            }
        }

        if (hitLatched == -1) {
            int manual = readManualShotInput();
            if (manual != -1) {
                hitLatched = manual;
            } else if (!gateTriggered) {
                double gate = readGateInches();
                if (gate > 0 && gate <= GATE_TRIGGER_IN) {
                    gateTriggered = true;
                    hitLatched = 1;
                }
            }
        }

        if (nowMs >= resultWindowDeadlineMs && hitLatched == -1) {
            hitLatched = 0;
        }

        if (!observationSent && shotFired && hitLatched != -1 && nowMs >= resultWindowDeadlineMs) {
            long latency = (fireCommandedAtMs >= 0 && shotDetectedMs >= 0) ? (shotDetectedMs - fireCommandedAtMs) : -1;
            plannerBridge.sendObsShot(
                    lastRange,
                    lastBattery,
                    (int) Math.round(rpmCommanded),
                    rpmAtFire,
                    hitLatched,
                    latency,
                    pose,
                    lastHeadingToTag
            );
            if (hitLatched == 1) {
                rangeTargetIn = clampRange(rangeTargetIn + STEP_IN);
            } else {
                rangeTargetIn = clampRange(rangeTargetIn);
            }
            shooter.closeLift();
            shooter.stopIntake();
            fireNowLatched = false;
            fireCommandedAtMs = -1;
            observationSent = true;
            transitionTo(State.DRIVE_AIM);
        }
    }

    private void beginFireSequence(long nowMs) {
        if (!fireSeqPrimed) {
            fireSeqPrimed = true;
            shooter.openLift();
            shooter.pulseIntake(nowMs);
            shotFired = false;
            gateTriggered = false;
        }
    }

    private void transitionTo(State newState) {
        if (state == newState) {
            return;
        }
        state = newState;
        if (newState == State.DRIVE_AIM) {
            planRequested = false;
            resultPrimed = false;
            fireSeqPrimed = false;
            fireNowLatched = false;
            fireCommandedAtMs = -1;
            currentPlan = ShotPlan.idle();
            pendingPlan = null;
        } else if (newState == State.REQUEST_PLAN) {
            planRequested = false;
            fireNowLatched = false;
            fireCommandedAtMs = -1;
            currentPlan = ShotPlan.idle();
            pendingPlan = null;
        } else if (newState == State.SPINUP_WAIT) {
            resultPrimed = false;
            fireSeqPrimed = false;
        } else if (newState == State.FIRE_SEQ) {
            resultPrimed = false;
        } else if (newState == State.RESULT_WAIT) {
            resultPrimed = false;
        }
    }

    private double readBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        if (voltageSensors != null) {
            for (VoltageSensor sensor : voltageSensors) {
                try {
                    double v = sensor.getVoltage();
                    if (v > 0) {
                        min = Math.min(min, v);
                    }
                } catch (Exception ignored) {
                }
            }
        }
        if (!Double.isFinite(min)) {
            min = 12.0;
        }
        return min;
    }

    private int readManualShotInput() {
        int result = -1;
        boolean g1A = gamepad1 != null && gamepad1.a;
        if (g1A && !g1APrev) {
            result = 1;
        }
        g1APrev = g1A;
        boolean g1B = gamepad1 != null && gamepad1.b;
        if (g1B && !g1BPrev) {
            result = 0;
        }
        g1BPrev = g1B;
        boolean g2A = gamepad2 != null && gamepad2.a;
        if (g2A && !g2APrev) {
            result = 1;
        }
        g2APrev = g2A;
        boolean g2B = gamepad2 != null && gamepad2.b;
        if (g2B && !g2BPrev) {
            result = 0;
        }
        g2BPrev = g2B;
        return result;
    }

    private double readGateInches() {
        if (gateSensor == null) {
            return -1.0;
        }
        try {
            double inches = gateSensor.getDistance(com.qualcomm.robotcore.external.navigation.DistanceUnit.INCH);
            return (Double.isNaN(inches) || inches <= 0) ? -1.0 : inches;
        } catch (Exception e) {
            return -1.0;
        }
    }

    private double clampRange(double value) {
        return Math.max(MIN_RANGE_IN, Math.min(MAX_RANGE_IN, value));
    }

    private void renderTelemetry(PedroShotNavigator.PoseSnapshot pose) {
        telemetry.addData("state", state);
        telemetry.addData("rangeTarget", String.format(Locale.US, "%.1f", rangeTargetIn));
        telemetry.addData("rangeActual", String.format(Locale.US, "%.1f", lastRange));
        telemetry.addData("rpmCmd", String.format(Locale.US, "%.0f", rpmCommanded));
        telemetry.addData("rpmMeas", String.format(Locale.US, "%.0f", rpmMeasured));
        telemetry.addData("bias", String.format(Locale.US, "%.1f", currentPlan.getRpmBias()));
        telemetry.addData("fireNow", fireNowLatched);
        telemetry.addData("loiter", currentPlan.isLoiter());
        telemetry.addData("batteryV", String.format(Locale.US, "%.2f", lastBattery));
        telemetry.addData("pose", String.format(Locale.US, "(%.1f, %.1f, %.1f°)",
                pose != null ? pose.xIn : 0.0,
                pose != null ? pose.yIn : 0.0,
                pose != null ? pose.headingDeg : 0.0));
        telemetry.addData("headingToTag", String.format(Locale.US, "%.1f", lastHeadingToTag));
        telemetry.addData("gate_in", String.format(Locale.US, "%.1f", readGateInches()));
        telemetry.update();
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.reset();
        }
    }
}
