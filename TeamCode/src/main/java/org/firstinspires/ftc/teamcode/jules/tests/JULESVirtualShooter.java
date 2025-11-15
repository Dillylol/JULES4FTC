package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.PwmControl.PwmRange;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import org.firstinspires.ftc.teamcode.common.BjornConstants;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.shot.MailboxConsumer;
import org.firstinspires.ftc.teamcode.jules.shot.ShooterController;
import org.firstinspires.ftc.teamcode.jules.shot.ShotPlannerBridge;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;

import java.lang.reflect.Method;
import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Virtual shooter mode that mimics the physical flywheel, intake and lift using software motors.
 *
 * <p>The program mirrors the structure of {@link JULESSimDrive} so it plugs into the normal
 * JULES telemetry/command pipeline. Instead of a drivetrain we expose three virtual motors that
 * use the real hardware names ("Wheel", "Wheel2", "Intake") plus the "Lift" servo. The motors
 * obey the shooter controller so remote tooling can request RPM set-points and fire commands
 * without real hardware attached.</p>
 */
@TeleOp(name = "JULES Virtual Shooter", group = "JULES")
public final class JULESVirtualShooter extends OpMode {

    // ---- JULES plumbing ----
    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;
    private int bridgePort = 58080;

    private JulesStreamBus.Subscription busSub;
    private Thread busPump;
    private final Queue<String> pendingCmds = new ConcurrentLinkedQueue<>();

    // ---- RL shot-planning bridge ----
    private ShotPlannerBridge rlBridge;
    private MailboxConsumer rlMailbox;  // currently unused but wired for future

    // ---- Virtual hardware ----
    private final VirtualMotorEx wheel = new VirtualMotorEx(BjornConstants.Motors.WHEEL, true);
    private final VirtualMotorEx wheel2 = new VirtualMotorEx(BjornConstants.Motors.WHEEL2, true);
    private final VirtualMotorEx intake = new VirtualMotorEx(BjornConstants.Motors.INTAKE, false);
    private final VirtualServo lift = new VirtualServo(BjornConstants.Servos.LIFT);

    private ShooterController shooter;

    // Real pack sensor (if present)
    private VoltageSensor hubVoltage;

    // ---- State ----
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastSnapshotMs = 0;
    private double lastHeartbeatMs = 0;
    private final JulesDataOrganizer dataOrganizer = JulesDataOrganizer.getInstance();
    private static final double BATTERY_MAX_V = 13.2;
    private static final double BATTERY_MIN_V = 10.9;
    private static final double BATTERY_LOAD_SAG_PER_SEC = 0.16;
    private static final double BATTERY_RECOVERY_PER_SEC = 0.05;
    private double batteryVoltage = 12.6;

    private int manualTargetRpm = 2400;
    private boolean spinRequested = false;
    private boolean lastFireButton = false;
    private boolean lastSpinToggle = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    private ShooterController.ShotMetrics lastShotMetrics;
    private long lastShotTimestampMs = 0L;
    private long shotSeq = 0L;
    private String lastShotId = null;
    private String sessionId = newSessionId();
    private static final String BOT_ID = "virtual_shooter";

    @Override
    public void init() {
        shooter = new ShooterController(wheel, wheel2, intake, lift);
        sessionId = newSessionId();
        shotSeq = 0L;
        lastShotId = null;

        // Grab the first available hub voltage sensor (real pack)
        try {
            for (VoltageSensor vs : hardwareMap.getAll(VoltageSensor.class)) {
                hubVoltage = vs;
                break;
            }
        } catch (Exception ignored) {
        }

        bridgeManager = JulesBridgeManager.getInstance();
        if (bridgeManager != null) {
            bridgeManager.prepare(hardwareMap.appContext);
            bridgePort = bridgeManager.getPort();
            streamBus = bridgeManager.getStreamBus();
        }

        // RL bridge: use the same RL server as the physical ShotTrainer
        rlBridge = new ShotPlannerBridge(hardwareMap.appContext);
        rlBridge.setBotId("virtual_shooter");
        rlBridge.setSessionId("default");  // TODO make configurable if needed
        rlBridge.connect();

        // Optional mailbox (not actively used yet here; full RL loop lives in ShotTrainer)
        rlMailbox = new MailboxConsumer(rlBridge, /* rpmProvider */ null);

        // Subscribe to the shared bus so we can receive commands even if Panels is offline.
        if (streamBus != null) {
            try {
                busSub = streamBus.subscribe();
                busPump = new Thread(() -> {
                    try {
                        for (;;) {
                            String line = busSub.take();
                            if (line == null) {
                                break;
                            }
                            JsonElement el = GsonCompat.parse(line);
                            if (el != null && el.isJsonObject()) {
                                JsonObject o = el.getAsJsonObject();
                                if ("cmd".equalsIgnoreCase(optString(o, "type"))) {
                                    JsonElement text = o.get("text");
                                    if (text != null) {
                                        if (text.isJsonObject()) {
                                            pendingCmds.offer(text.getAsJsonObject().toString());
                                        } else if (text.isJsonPrimitive()) {
                                            pendingCmds.offer(text.getAsString());
                                        }
                                    }
                                }
                            }
                        }
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                    } catch (Throwable ignored) {
                        // ignore bus errors; sim keeps running locally
                    }
                }, "JulesSimShooterBus");
                busPump.setDaemon(true);
                busPump.start();
            } catch (Throwable ignored) {
            }
        }

        telemetry.addLine("JULES Virtual Shooter ready (A=spin, B=stop, RB=fire)");
        telemetry.addData("bridge", bridgeManager != null ? "connected" : "offline");
        telemetry.update();
        loopTimer.reset();

        // publish the sim pack separately so the main vitals.battery_v can stay real
        dataOrganizer.recordTelemetry("sim.battery_v", batteryVoltage);
    }

    @Override
    public void start() {
        publishHeartbeat();
        publishSnapshot();
    }

    @Override
    public void loop() {
        double nowMs = loopTimer.milliseconds();
        double dt = Math.max(0.001, (nowMs - lastSnapshotMs) / 1000.0);

        handleIncomingCommand((long) nowMs);

        // Manual controls ---------------------------------------------------
        boolean spinToggle = gamepad1.a;
        if (spinToggle && !lastSpinToggle) {
            spinRequested = !spinRequested;
        }
        lastSpinToggle = spinToggle;

        boolean stopBtn = gamepad1.b;
        if (stopBtn) {
            spinRequested = false;
            shooter.stop((long) nowMs);
        }

        boolean dpadUp = gamepad1.dpad_up;
        boolean dpadDown = gamepad1.dpad_down;
        if (dpadUp && !lastDpadUp) {
            manualTargetRpm = Math.min(3000, manualTargetRpm + 50);
        }
        if (dpadDown && !lastDpadDown) {
            manualTargetRpm = Math.max(1200, manualTargetRpm - 50);
        }
        lastDpadUp = dpadUp;
        lastDpadDown = dpadDown;

        double triggerSpin = gamepad1.right_trigger;
        boolean spinWhileHeld = triggerSpin > 0.35;
        boolean shouldSpin = spinRequested || spinWhileHeld;

        if (shouldSpin) {
            shooter.setTargetRpm(manualTargetRpm, (long) nowMs);
        } else {
            shooter.stop((long) nowMs);
        }

        boolean fireBtn = gamepad1.right_bumper;
        if (fireBtn && !lastFireButton) {
            if (shooter.fire((long) nowMs)) {
                publishCmdStatus("fire", "accepted", null);
            } else {
                publishCmdStatus("fire", "rejected", null);
            }
        }
        lastFireButton = fireBtn;

        shooter.update((long) nowMs);
        ShooterController.ShotMetrics metrics = shooter.pollShotMetrics();
        if (metrics != null) {
            lastShotMetrics = metrics;
            lastShotTimestampMs = System.currentTimeMillis();
            publishShotEvent(metrics);
        }

        wheel.update(dt);
        wheel2.update(dt);
        intake.update(dt);
        lift.update();
        updateBatteryModel(dt, shouldSpin || shooter.isUnderLoad());

        if (nowMs - lastHeartbeatMs >= 1000.0) {
            publishHeartbeat();
            lastHeartbeatMs = nowMs;
        }
        if (nowMs - lastSnapshotMs >= 120.0) {
            publishSnapshot();
            lastSnapshotMs = nowMs;
        }

        telemetry.addData("targetRpm", shooter.getTargetRpm());
        telemetry.addData("measuredRpm", String.format(Locale.US, "%.0f", shooter.getMeasuredRpm()));
        telemetry.addData("ready", shooter.isReady((long) nowMs));
        telemetry.addData("lockedOut", shooter.isLockedOut((long) nowMs));
        telemetry.addData("manualTarget", manualTargetRpm);
        telemetry.addData("spinRequested", shouldSpin);
        if (lastShotMetrics != null) {
            telemetry.addData("lastShotRpm", lastShotMetrics.rpmAtFire);
            telemetry.addData("lastShotLatencyMs", lastShotMetrics.timeToReadyMs);
        }
        telemetry.addData("wheel", wheel.debug());
        telemetry.addData("wheel2", wheel2.debug());
        telemetry.addData("intake", intake.debug());
        telemetry.addData("lift", lift.debug());
        telemetry.update();
    }

    @Override
    public void stop() {
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
        // rlMailbox has no close(); just let it be GC'd
        if (rlBridge != null) {
            try {
                rlBridge.close();
            } catch (Exception ignored) {
            }
            rlBridge = null;
        }
    }

    // ---------------------------------------------------------------------
    // Command handling
    // ---------------------------------------------------------------------
    private void handleIncomingCommand(long nowMs) {
        String raw;
        while ((raw = mailboxOrBusTake()) != null) {
            try {
                JsonElement parsed = GsonCompat.parse(raw);
                if (parsed != null && parsed.isJsonObject()) {
                    JsonObject obj = parsed.getAsJsonObject();
                    if (obj.has("cmd")) {
                        String cmd = optString(obj, "cmd");
                        if (cmd != null) {
                            if (handleStructuredCommand(cmd.toLowerCase(Locale.US), obj, nowMs)) {
                                continue;
                            }
                        }
                    }
                    if (obj.has("text") && obj.get("text").isJsonPrimitive()) {
                        String text = obj.get("text").getAsString();
                        handlePlainText(text.toLowerCase(Locale.US), nowMs);
                        continue;
                    }
                }
                handlePlainText(raw.toLowerCase(Locale.US), nowMs);
            } catch (Throwable ignored) {
            }
        }
    }

    private boolean handleStructuredCommand(String cmd, JsonObject payload, long nowMs) {
        switch (cmd) {
            case "spin":
            case "set_rpm": {
                double rpm = optDouble(payload, new String[]{"rpm", "target", "value"}, manualTargetRpm);
                int newTarget = (int) Math.round(rpm);
                manualTargetRpm = Range.clip(newTarget, 1200, 3000);
                shooter.setTargetRpm(manualTargetRpm, nowMs);
                spinRequested = true;
                publishCmdStatus("spin", "accepted", payload);
                return true;
            }
            case "stop": {
                spinRequested = false;
                shooter.stop(nowMs);
                publishCmdStatus("stop", "accepted", payload);
                return true;
            }
            case "fire": {
                boolean ok = shooter.fire(nowMs);
                publishCmdStatus("fire", ok ? "accepted" : "rejected", payload);
                return true;
            }
            case "intake": {
                double power = Range.clip(optDouble(payload, new String[]{"power", "value"}, 0.0), -1.0, 1.0);
                intake.setPower(power);
                publishCmdStatus("intake", "accepted", payload);
                return true;
            }
            default:
                return false;
        }
    }

    private void handlePlainText(String lower, long nowMs) {
        if (lower.contains("fire")) {
            boolean ok = shooter.fire(nowMs);
            publishCmdStatus("fire", ok ? "accepted" : "rejected", null);
            return;
        }
        if (lower.contains("stop")) {
            spinRequested = false;
            shooter.stop(nowMs);
            publishCmdStatus("stop", "accepted", null);
            return;
        }
        if (lower.contains("spin") || lower.contains("rpm")) {
            double rpm = parseMagnitude(lower, manualTargetRpm);
            manualTargetRpm = Range.clip((int) Math.round(rpm), 1200, 3000);
            shooter.setTargetRpm(manualTargetRpm, nowMs);
            spinRequested = true;
            publishCmdStatus("spin", "accepted", null);
        }
    }

    private String mailboxOrBusTake() {
        String queued = pendingCmds.poll();
        if (queued != null) {
            return queued;
        }

        // For now we do NOT consume RL mailbox here; plans are handled in ShotTrainerOpMode.

        try {
            for (String m : new String[]{"getAndClear", "take", "poll", "consume", "next"}) {
                try {
                    Method mm = JulesCommand.class.getMethod(m);
                    Object out = mm.invoke(null);
                    if (out != null) {
                        return out.toString();
                    }
                } catch (NoSuchMethodException ignored) {
                }
            }
            try {
                Method get = JulesCommand.class.getMethod("get");
                Object out = get.invoke(null);
                if (out == null) {
                    return null;
                }
                try {
                    JulesCommand.class.getMethod("clear").invoke(null);
                } catch (NoSuchMethodException e) {
                    JulesCommand.class.getMethod("setCommand", String.class).invoke(null, (String) null);
                }
                return out.toString();
            } catch (NoSuchMethodException ignored) {
            }
        } catch (Throwable ignored) {
        }
        return null;
    }

    private static String optString(JsonObject o, String key) {
        if (o == null || key == null) {
            return null;
        }
        if (!o.has(key)) {
            return null;
        }
        JsonElement el = o.get(key);
        return el.isJsonPrimitive() ? el.getAsString() : null;
    }

    private static double optDouble(JsonObject o, String[] keys, double def) {
        if (o == null) {
            return def;
        }
        for (String key : keys) {
            try {
                if (o.has(key) && o.get(key).isJsonPrimitive()) {
                    return o.get(key).getAsDouble();
                }
            } catch (Exception ignored) {
            }
        }
        return def;
    }

    private static double parseMagnitude(String text, double def) {
        try {
            String[] parts = text.replaceAll("[^0-9.+-]", " ").trim().split("\\s+");
            if (parts.length == 0 || parts[0].isEmpty()) {
                return def;
            }
            return Double.parseDouble(parts[0]);
        } catch (Exception e) {
            return def;
        }
    }

    // ---------------------------------------------------------------------
    // Publishing helpers
    // ---------------------------------------------------------------------
    private void publishHeartbeat() {
        JsonObject hb = new JsonObject();
        hb.addProperty("type", "heartbeat");
        hb.addProperty("ts_ms", System.currentTimeMillis());
        hb.addProperty("uptime_ms", (long) loopTimer.milliseconds());
        hb.addProperty("active_opmode", "JULES Virtual Shooter");
        hb.addProperty("session_id", sessionId);
        hb.addProperty("bot_id", BOT_ID);

        double packV = currentPackVoltage();
        if (Double.isFinite(packV)) {
            hb.addProperty("battery_v", packV);
        }
        hb.addProperty("port", bridgePort);
        busPublish(hb.toString());
    }

    private void publishSnapshot() {
        JsonObject snap = new JsonObject();
        snap.addProperty("type", "snapshot");
        snap.addProperty("ts_ms", System.currentTimeMillis());
        snap.addProperty("session_id", sessionId);
        snap.addProperty("bot_id", BOT_ID);

        double packV = currentPackVoltage();
        if (Double.isFinite(packV)) {
            snap.addProperty("battery_v", packV);
        }

        JsonObject motors = new JsonObject();
        motors.add(wheel.name, wheel.toJson());
        motors.add(wheel2.name, wheel2.toJson());
        motors.add(intake.name, intake.toJson());

        JsonObject sim = new JsonObject();
        sim.add("motors", motors);
        sim.add("lift", lift.toJson());
        sim.addProperty("target_rpm", shooter.getTargetRpm());
        sim.addProperty("measured_rpm", shooter.getMeasuredRpm());
        sim.addProperty("ready", shooter.isReady((long) loopTimer.milliseconds()));
        sim.addProperty("locked_out", shooter.isLockedOut((long) loopTimer.milliseconds()));
        sim.addProperty("spin_requested", spinRequested);
        sim.addProperty("battery_v", batteryVoltage);  // virtual pack for sim analysis

        if (lastShotMetrics != null) {
            JsonObject shot = new JsonObject();
            shot.addProperty("rpm_at_fire", lastShotMetrics.rpmAtFire);
            shot.addProperty("ready_latency_ms", lastShotMetrics.timeToReadyMs);
            shot.addProperty("fire_ts", lastShotMetrics.fireTimestampMs);
            shot.addProperty("recorded_ts_ms", lastShotTimestampMs);
            if (lastShotId != null) {
                shot.addProperty("shot_id", lastShotId);
            }
            sim.add("last_shot", shot);
        }

        snap.add("sim", sim);
        busPublish(snap.toString());
    }

    private static String newSessionId() {
        return String.format(Locale.US, "virt-%08x", System.currentTimeMillis() & 0xffffffffL);
    }

    private String nextShotId() {
        shotSeq += 1L;
        return sessionId + "-" + shotSeq;
    }

    private double currentPackVoltage() {
        if (hubVoltage != null) {
            try {
                double v = hubVoltage.getVoltage();
                if (Double.isFinite(v)) {
                    return v;
                }
            } catch (Exception ignored) {
            }
        }
        return batteryVoltage;
    }

    private void publishShotEvent(ShooterController.ShotMetrics metrics) {
        JsonObject ev = new JsonObject();
        ev.addProperty("type", "shot");
        ev.addProperty("ts_ms", System.currentTimeMillis());
        String shotId = nextShotId();
        lastShotId = shotId;
        ev.addProperty("shot_id", shotId);
        ev.addProperty("session_id", sessionId);
        ev.addProperty("bot_id", BOT_ID);
        double packV = currentPackVoltage();
        if (Double.isFinite(packV)) {
            ev.addProperty("battery_v", packV);
        }
        ev.addProperty("rpm_at_fire", metrics.rpmAtFire);
        ev.addProperty("ready_latency_ms", metrics.timeToReadyMs);
        ev.addProperty("fire_command_ms", metrics.fireTimestampMs);
        JsonObject context = new JsonObject();
        context.addProperty("manual_target_rpm", manualTargetRpm);
        context.addProperty("target_rpm", shooter.getTargetRpm());
        context.addProperty("measured_rpm", shooter.getMeasuredRpm());
        context.addProperty("spin_requested", spinRequested);
        context.addProperty("locked_out", shooter.isLockedOut((long) loopTimer.milliseconds()));
        if (Double.isFinite(packV)) {
            context.addProperty("battery_v", packV);
        }
        context.addProperty("sim_battery_v", batteryVoltage);
        ev.add("context", context);
        busPublish(ev.toString());
    }

    private void updateBatteryModel(double dt, boolean underLoad) {
        double load = Math.abs(wheel.getPower()) + Math.abs(wheel2.getPower()) + Math.abs(intake.getPower());
        if (underLoad) {
            load += 1.2; // approximate flywheel surge
        }
        double drop = load * BATTERY_LOAD_SAG_PER_SEC * dt;
        if (drop > 0) {
            batteryVoltage = Math.max(BATTERY_MIN_V, batteryVoltage - drop);
        }
        if (!underLoad && load < 0.35) {
            double recover = (0.35 - load) * BATTERY_RECOVERY_PER_SEC * dt;
            if (recover > 0) {
                batteryVoltage = Math.min(BATTERY_MAX_V, batteryVoltage + recover);
            }
        }
        dataOrganizer.recordTelemetry("sim.battery_v", batteryVoltage);
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
        if (json == null) {
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
                return shared;
            }
        }
        return streamBus;
    }

    // ---------------------------------------------------------------------
    // Virtual motor implementation (DcMotorEx stub)
    // ---------------------------------------------------------------------
    private static final class VirtualMotorEx implements DcMotorEx {
        private static final double TICKS_PER_REV = 28.0;
        private static final double MAX_RPM = 3000.0;
        private static final double MAX_TICKS_PER_SECOND = (MAX_RPM * TICKS_PER_REV) / 60.0;

        final String name;
        private final boolean velocityMotor;

        private DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        private DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT;
        private DcMotor.RunMode mode = DcMotor.RunMode.RUN_USING_ENCODER;
        private MotorConfigurationType motorType = null;

        private double targetVelocity = 0.0;
        private double velocity = 0.0;
        private double targetPower = 0.0;
        private double power = 0.0;
        private double position = 0.0;
        private int targetPosition = 0;
        private int targetPositionTolerance = 5;

        private boolean enabled = true;
        private double currentAlert = Double.POSITIVE_INFINITY;
        private double currentDraw = 0.0;

        VirtualMotorEx(String name, boolean velocityMotor) {
            this.name = name;
            this.velocityMotor = velocityMotor;
        }

        void update(double dt) {
            double tau = velocityMotor ? 0.08 : 0.05;
            double alpha = Range.clip(dt / Math.max(1e-3, tau), 0.0, 1.0);
            velocity += alpha * (targetVelocity - velocity);
            power += alpha * (targetPower - power);
            position += velocity * dt;
            currentDraw = Math.abs(power) * 6.0;
        }

        JsonObject toJson() {
            JsonObject o = new JsonObject();
            o.addProperty("target_velocity", targetVelocity);
            o.addProperty("velocity", velocity);
            o.addProperty("target_power", targetPower);
            o.addProperty("power", power);
            o.addProperty("position", position);
            return o;
        }

        String debug() {
            return String.format(Locale.US, "pow=%.2f vel=%.0f tgt=%.0f", power, velocity, targetVelocity);
        }

        // DcMotorSimple ------------------------------------------------------------------
        @Override
        public void setDirection(Direction direction) {
            this.direction = direction == null ? DcMotorSimple.Direction.FORWARD : direction;
        }

        @Override
        public Direction getDirection() {
            return direction;
        }

        @Override
        public void setPower(double power) {
            double clipped = Range.clip(power, -1.0, 1.0);
            targetPower = clipped;
            if (!velocityMotor) {
                targetVelocity = clipped * MAX_TICKS_PER_SECOND * (direction == Direction.FORWARD ? 1 : -1);
            }
        }

        @Override
        public double getPower() {
            return power;
        }

        @Override
        public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
            this.zeroPowerBehavior = zeroPowerBehavior;
        }

        @Override
        public ZeroPowerBehavior getZeroPowerBehavior() {
            return zeroPowerBehavior;
        }

        // DcMotor -----------------------------------------------------------------------
        @Override
        public void setMotorType(MotorConfigurationType motorType) {
            this.motorType = motorType;
        }

        @Override
        public MotorConfigurationType getMotorType() {
            return motorType;
        }

        @Override
        public DcMotorController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }

        @Override
        public int getCurrentPosition() {
            return (int) Math.round(position);
        }

        @Override
        public void setTargetPosition(int position) {
            this.targetPosition = position;
        }

        @Override
        public int getTargetPosition() {
            return targetPosition;
        }

        @Override
        public boolean isBusy() {
            return Math.abs(targetPosition - getCurrentPosition()) > targetPositionTolerance;
        }

        @Override
        public void setMode(RunMode mode) {
            this.mode = mode == null ? DcMotor.RunMode.RUN_WITHOUT_ENCODER : mode;
        }

        @Override
        public RunMode getMode() {
            return mode;
        }

        @Override
        public void setPowerFloat() {
            targetPower = 0.0;
        }

        @Override
        public boolean getPowerFloat() {
            return Math.abs(power) < 1e-3;
        }

        // DcMotorEx ---------------------------------------------------------------------
        @Override
        public void setVelocity(double angularRate) {
            double scaled = Range.clip(angularRate, -MAX_TICKS_PER_SECOND, MAX_TICKS_PER_SECOND);
            targetVelocity = scaled * (direction == Direction.FORWARD ? 1 : -1);
            if (velocityMotor) {
                targetPower = Range.clip(targetVelocity / MAX_TICKS_PER_SECOND, -1.0, 1.0);
            }
        }

        @Override
        public void setVelocity(double angularRate, AngleUnit unit) {
            double ticksPerSecond = unit == AngleUnit.DEGREES
                    ? angularRate * TICKS_PER_REV / 360.0
                    : angularRate * TICKS_PER_REV / (2 * Math.PI);
            setVelocity(ticksPerSecond);
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public double getVelocity(AngleUnit unit) {
            if (unit == AngleUnit.DEGREES) {
                return (velocity / TICKS_PER_REV) * 360.0;
            }
            return (velocity / TICKS_PER_REV) * (2 * Math.PI);
        }

        @Override
        public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
            // no-op
        }

        @Override
        public PIDCoefficients getPIDCoefficients(RunMode mode) {
            return new PIDCoefficients(0, 0, 0);
        }

        @Override
        public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidfCoefficients) {
            // no-op
        }

        @Override
        public PIDFCoefficients getPIDFCoefficients(RunMode mode) {
            return new PIDFCoefficients(0, 0, 0, 0);
        }

        @Override
        public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
            // no-op
        }

        @Override
        public void setPositionPIDFCoefficients(double p) {
            // no-op
        }

        @Override
        public void setTargetPositionTolerance(int tolerance) {
            this.targetPositionTolerance = tolerance;
        }

        @Override
        public int getTargetPositionTolerance() {
            return targetPositionTolerance;
        }

        @Override
        public void setMotorEnable() {
            enabled = true;
        }

        @Override
        public void setMotorDisable() {
            enabled = false;
            targetPower = 0.0;
            targetVelocity = 0.0;
        }

        @Override
        public boolean isMotorEnabled() {
            return enabled;
        }

        @Override
        public void setCurrentAlert(double current, CurrentUnit unit) {
            if (unit == CurrentUnit.AMPS) {
                currentAlert = current;
            } else {
                currentAlert = unit.toAmps(current);
            }
        }

        @Override
        public double getCurrentAlert(CurrentUnit unit) {
            double amps = Double.isFinite(currentAlert) ? currentAlert : 0.0;
            return unit == CurrentUnit.AMPS ? amps : unit.convert(amps, CurrentUnit.AMPS);
        }

        @Override
        public boolean isOverCurrent() {
            return currentAlert < Double.POSITIVE_INFINITY && currentDraw > currentAlert;
        }

        @Override
        public double getCurrent(CurrentUnit unit) {
            double amps = currentDraw;
            return unit == CurrentUnit.AMPS ? amps : unit.convert(amps, CurrentUnit.AMPS);
        }

        // HardwareDevice -----------------------------------------------------------------
        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "VirtualMotor(" + name + ")";
        }

        @Override
        public String getConnectionInfo() {
            return "virtual";
        }

        @Override
        public int getVersion() {
            return 1;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
            // nothing
        }

        @Override
        public void close() {
            // nothing
        }
    }

    // ---------------------------------------------------------------------
    // Virtual servo implementation
    // ---------------------------------------------------------------------
    private static final class VirtualServo implements Servo, PwmControl {
        final String name;
        private Direction direction = Direction.FORWARD;
        private double position = BjornConstants.Servos.LIFT_LOWERED;
        private PwmRange pwmRange = new PwmRange(500, 2500);
        private boolean pwmEnabled = true;

        VirtualServo(String name) {
            this.name = name;
        }

        void update() {
            // nothing dynamic yet; placeholder for future inertia model
        }

        JsonObject toJson() {
            JsonObject o = new JsonObject();
            o.addProperty("position", position);
            o.addProperty("direction", direction.toString());
            o.addProperty("pwm_enabled", pwmEnabled);
            return o;
        }

        String debug() {
            return String.format(Locale.US, "pos=%.2f dir=%s", position, direction);
        }

        @Override
        public void setDirection(Direction direction) {
            this.direction = direction == null ? Direction.FORWARD : direction;
        }

        @Override
        public Direction getDirection() {
            return direction;
        }

        @Override
        public void setPosition(double position) {
            double clipped = Range.clip(position, 0.0, 1.0);
            this.position = direction == Direction.FORWARD ? clipped : (1.0 - clipped);
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public void scaleRange(double min, double max) {
            pwmRange = new PwmRange(min, max);
        }

        @Override
        public void setPwmRange(PwmRange range) {
            if (range != null) {
                pwmRange = range;
            }
        }

        @Override
        public PwmRange getPwmRange() {
            return pwmRange;
        }

        @Override
        public void setPwmEnable() {
            pwmEnabled = true;
        }

        @Override
        public void setPwmDisable() {
            pwmEnabled = false;
        }

        @Override
        public boolean isPwmEnabled() {
            return pwmEnabled;
        }

        @Override
        public ServoController getController() {
            return null;
        }

        @Override
        public int getPortNumber() {
            return 0;
        }

        @Override
        public Manufacturer getManufacturer() {
            return Manufacturer.Other;
        }

        @Override
        public String getDeviceName() {
            return "VirtualServo(" + name + ")";
        }

        @Override
        public String getConnectionInfo() {
            return "virtual";
        }

        @Override
        public int getVersion() {
            return 1;
        }

        @Override
        public void resetDeviceConfigurationForOpMode() {
            // nothing
        }

        @Override
        public void close() {
            // nothing
        }
    }
}
