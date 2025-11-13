package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;

// Optional Panels libs (we guard usage via reflection-compatible calls)
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.gamepad.PanelsGamepad;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * JULESSimDrive v3 — virtual drivetrain with JULES hooks + Panels gamepad.
 *
 * Key fixes vs prior:
 * - Uses shared JulesBridgeManager (no port rebind, taps existing server).
 * - Robust JSON command handling: accepts {"type":"cmd","text":"{...}"} OR object in text.
 * - Fallback subscription to StreamBus: consumes {type:"cmd", text: ...} events if mailbox is empty.
 * - Panels gamepad integrated exactly like your working OLD program (INSTANCE singletons).
 */
@TeleOp(name = "JULES SimDrive v3", group = "JULES")
public class JULESSimDrive extends OpMode {

    // ---- JULES plumbing (shared) ----
    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;
    private int bridgePort = 58080; // for telemetry only
    private final JulesDataOrganizer dataOrganizer = JulesDataOrganizer.getInstance();

    // Optional bus subscription fallback
    private JulesStreamBus.Subscription busSub;
    private Thread busPump;
    private final Queue<String> pendingCmds = new ConcurrentLinkedQueue<>();

    // ---- Sim state ----
    private final VirtualMotor lf = new VirtualMotor("lf");
    private final VirtualMotor rf = new VirtualMotor("rf");
    private final VirtualMotor lr = new VirtualMotor("lr");
    private final VirtualMotor rr = new VirtualMotor("rr");

    private static final double BATTERY_MAX_V = 13.2;
    private static final double BATTERY_MIN_V = 10.8;
    private static final double BATTERY_LOAD_SAG_PER_SEC = 0.08;
    private static final double BATTERY_RECOVERY_PER_SEC = 0.03;
    private double batteryVoltage = 12.6;

    private double xIn = 0, yIn = 0, headingDeg = 0; // toy pose

    // rate keeping
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastSnapshotMs = 0;
    private double lastHeartbeatMs = 0;

    // manual/programmatic
    private boolean manualMode = true;
    private boolean lastToggleBtn = false;

    // command timing
    private boolean cmdActive = false;
    private double cmdEndMs = 0;
    private String  cmdName = null;
    private String  lastCmdConsumed = "(none)";

    // ---- Panels (reflect-friendly use of Kotlin singletons) ----
    private Object panelsTelemetryObj;                   // PanelsTelemetry.INSTANCE.getTelemetry()
    private Object panelsMgr1;                           // PanelsGamepad.INSTANCE.getFirstManager()
    private Object panelsMgr2;                           // PanelsGamepad.INSTANCE.getSecondManager()
    private Method asCombinedMethod;                     // asCombinedFTCGamepad(Gamepad)

    @Override
    public void init() {
        // 1) Attach to existing JULES bridge/bus
        bridgeManager = JulesBridgeManager.getInstance();
        if (bridgeManager != null) {
            bridgeManager.prepare(hardwareMap.appContext); // ensure context set
            bridgePort = bridgeManager.getPort();
            streamBus = bridgeManager.getStreamBus();
        }

        // 1a) Subscribe to bus for cmd events as a fallback path
        if (streamBus != null) {
            try {
                busSub = streamBus.subscribe();
                busPump = new Thread(() -> {
                    try {
                        for (;;) {
                            String line = busSub.take();
                            if (line == null) break;
                            JsonElement el = GsonCompat.parse(line);
                            if (el != null && el.isJsonObject()) {
                                JsonObject o = el.getAsJsonObject();
                                String t = optString(o, "type");
                                if (t != null && t.equalsIgnoreCase("cmd")) {
                                    // Expect {type:"cmd", text: <string or object>}
                                    if (o.has("text")) {
                                        if (o.get("text").isJsonObject()) {
                                            pendingCmds.offer(o.getAsJsonObject("text").toString());
                                        } else if (o.get("text").isJsonPrimitive()) {
                                            pendingCmds.offer(o.get("text").getAsString());
                                        }
                                    }
                                }
                            }
                        }
                    } catch (InterruptedException ie) {
                        Thread.currentThread().interrupt();
                    } catch (Throwable ignored) { }
                }, "JulesSimBusPump");
                busPump.setDaemon(true);
                busPump.start();
            } catch (Throwable ignored) { }
        }

        // 2) Panels telemetry + gamepad managers (Kotlin singletons via Java)
        try { panelsTelemetryObj = PanelsTelemetry.INSTANCE.getTelemetry(); } catch (Throwable t) { panelsTelemetryObj = null; }
        try {
            panelsMgr1 = PanelsGamepad.INSTANCE.getFirstManager();
            panelsMgr2 = PanelsGamepad.INSTANCE.getSecondManager();
            if (panelsMgr1 != null) {
                asCombinedMethod = panelsMgr1.getClass().getMethod("asCombinedFTCGamepad", Gamepad.class);
            }
        } catch (Throwable ignored) {
            panelsMgr1 = null; panelsMgr2 = null; asCombinedMethod = null;
        }

        ptInfo("JULES SimDrive v3 online (manual='X' to toggle)");
        if (bridgeManager != null) {
            String adv = bridgeManager.getAdvertiseLine();
            if (adv != null) { telemetry.addLine(adv); ptInfo(adv); }
            if (streamBus == null) {
                String offline = "JULES bus offline — run 'JULES: Enable & Status'.";
                telemetry.addLine(offline); ptInfo(offline);
            }
        } else {
            String offline = "JULES bridge unavailable.";
            telemetry.addLine(offline); ptInfo(offline);
        }
        telemetry.update();
        loopTimer.reset();
        dataOrganizer.updateBatteryOverride(batteryVoltage);
    }

    @Override public void start() { publishHeartbeat(); publishSnapshot(); }

    @Override
    public void loop() {
        final double nowMs = loopTimer.milliseconds();
        final double dt = Math.max(0.001, (nowMs - lastSnapshotMs) / 1000.0);

        // 1) Commands (mailbox + bus fallback)
        handleIncomingCommand();

        // 1b) Timed command stop
        if (cmdActive && nowMs >= cmdEndMs) {
            setDrivePowers(0,0,0,0);
            publishCmdStatus(cmdName, "completed", null);
            cmdActive = false; cmdName = null; manualMode = true;
        }

        // 2) Manual drive (Panels-combined preferred)
        boolean toggle = readButton("x");
        if (toggle && !lastToggleBtn) manualMode = !manualMode;
        lastToggleBtn = toggle;

        if (manualMode) {
            double fwd = -readAxis("left_stick_y");
            double str =  readAxis("left_stick_x");
            double trn =  readAxis("right_stick_x");
            setDriveFromVectors(fwd, str, trn);
        }

        // 3) Motor dynamics
        lf.update(dt); rf.update(dt); lr.update(dt); rr.update(dt);
        updateBatteryModel(dt);

        // 4) Toy kinematics
        stepKinematics(dt);

        // 5) Telemetry
        if (nowMs - lastHeartbeatMs >= 1000) { publishHeartbeat(); lastHeartbeatMs = nowMs; }
        if (nowMs - lastSnapshotMs  >= 100)  { publishSnapshot();  lastSnapshotMs  = nowMs; }

        String pose = String.format(Locale.US, "x=%.1f y=%.1f θ=%.1f°", xIn, yIn, headingDeg);
        telemetry.addData("mode", manualMode ? "manual" : "programmatic");
        telemetry.addData("cmdActive", cmdActive);
        telemetry.addData("lastCmd", lastCmdConsumed);
        telemetry.addData("pose", pose);
        telemetry.addData("lf", lf.debug());
        telemetry.addData("rf", rf.debug());
        telemetry.addData("lr", lr.debug());
        telemetry.addData("rr", rr.debug());
        telemetry.update();

        ptDebug("mode=" + (manualMode ? "manual" : "programmatic") + ", cmdActive=" + cmdActive);
        ptDebug("lastCmd " + lastCmdConsumed);
        ptDebug("pose " + pose);
        ptDebug("lf " + lf.debug());
        ptDebug("rf " + rf.debug());
        ptDebug("lr " + lr.debug());
        ptDebug("rr " + rr.debug());
        ptUpdate();
    }

    @Override
    public void stop() {
        dataOrganizer.clearBatteryOverride();
        try { if (busSub != null) { busSub.close(); busSub = null; } } catch (Throwable ignored) {}
        try { if (busPump != null) { busPump.interrupt(); busPump = null; } } catch (Throwable ignored) {}
    }

    // ------------------------------------------------------------
    // Command handling (robust to text-as-string and text-as-object)
    // ------------------------------------------------------------
    private void handleIncomingCommand() {
        try {
            String raw = mailboxOrBusTake();
            if (raw == null) return;
            raw = raw.trim();

            // Prefer structured JSON first
            if (raw.startsWith("{") && raw.endsWith("}")) {
                JsonElement el = GsonCompat.parse(raw);
                if (el != null && el.isJsonObject()) {
                    JsonObject root = el.getAsJsonObject();

                    // Unwrap typical server frame: {"type":"cmd","text": <OBJECT|STRING>}
                    if (root.has("type") && root.has("text")) {
                        JsonElement txt = root.get("text");
                        if (txt.isJsonObject()) {
                            root = txt.getAsJsonObject();
                        } else if (txt.isJsonPrimitive() && txt.getAsJsonPrimitive().isString()) {
                            JsonElement inner = GsonCompat.parse(txt.getAsString());
                            if (inner != null && inner.isJsonObject()) {
                                root = inner.getAsJsonObject();
                            } else {
                                // could be plain-text command inside text
                                handlePlainText(txt.getAsString().toLowerCase(Locale.US));
                                lastCmdConsumed = txt.getAsString();
                                return;
                            }
                        }
                    }

                    // Allow {"command":"..."} style too
                    if (root.has("command") && root.get("command").isJsonPrimitive()) {
                        String cmdStr = root.get("command").getAsString();
                        JsonElement inner = GsonCompat.parse(cmdStr);
                        if (inner != null && inner.isJsonObject()) {
                            root = inner.getAsJsonObject();
                        } else {
                            handlePlainText(cmdStr.toLowerCase(Locale.US));
                            lastCmdConsumed = cmdStr;
                            return;
                        }
                    }

                    // Now expect { label?, name? | type?, args? }
                    String name = optString(root, "name");
                    String type = (name == null) ? optString(root, "type") : null;
                    String verb = (name != null) ? name : type;
                    if (verb != null) verb = verb.toLowerCase(Locale.US);

                    if ("drive".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject() ? root.getAsJsonObject("args") : new JsonObject();
                        double p  = optDouble(args, new String[]{"p","power","fwd","forward"}, 0.0);
                        double t  = optDouble(args, new String[]{"t","turn"}, 0.0);
                        double s  = optDouble(args, new String[]{"s","strafe","x"}, 0.0);
                        int ms    = (int)Math.round(optDouble(args, new String[]{"duration_ms","ms","duration"}, 0.0));
                        manualMode = false; setDriveFromVectors(p, s, t); publishCmdStatus("drive","started", args);
                        if (ms > 0) { cmdActive = true; cmdEndMs = loopTimer.milliseconds() + ms; cmdName = "drive"; }
                        lastCmdConsumed = root.toString();
                        return;
                    }
                    if ("strafe".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject() ? root.getAsJsonObject("args") : new JsonObject();
                        double s = optDouble(args, new String[]{"speed","s","strafe","x"}, 0.0);
                        int ms   = (int)Math.round(optDouble(args, new String[]{"duration_ms","ms","duration"}, 0.0));
                        manualMode = false; setDriveFromVectors(0, s, 0); publishCmdStatus("strafe","started", args);
                        if (ms > 0) { cmdActive = true; cmdEndMs = loopTimer.milliseconds() + ms; cmdName = "strafe"; }
                        lastCmdConsumed = root.toString();
                        return;
                    }
                    if ("turn".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject() ? root.getAsJsonObject("args") : new JsonObject();
                        double sp = optDouble(args, new String[]{"speed","t","turn"}, 0.0);
                        int ms    = (int)Math.round(optDouble(args, new String[]{"duration_ms","ms","duration"}, 0.0));
                        manualMode = false; setDriveFromVectors(0, 0, sp); publishCmdStatus("turn","started", args);
                        if (ms > 0) { cmdActive = true; cmdEndMs = loopTimer.milliseconds() + ms; cmdName = "turn"; }
                        lastCmdConsumed = root.toString();
                        return;
                    }
                    if ("stop".equals(verb)) {
                        manualMode = false; setDrivePowers(0,0,0,0); publishCmdStatus("stop","completed", root);
                        cmdActive = false; cmdName = null; lastCmdConsumed = root.toString();
                        return;
                    }

                    // Unknown JSON → try plain text in a text field
                    if (root.has("text") && root.get("text").isJsonPrimitive()) {
                        String s = root.get("text").getAsString();
                        handlePlainText(s.toLowerCase(Locale.US));
                        lastCmdConsumed = s;
                        return;
                    }
                }
            }

            // Plain text fallback
            handlePlainText(raw.toLowerCase(Locale.US));
            lastCmdConsumed = raw;
        } catch (Throwable ignored) { }
    }

    private void handlePlainText(String lower) {
        manualMode = false;
        double p = parseMagnitude(lower, 0.6);
        if (lower.contains("stop")) {
            setDrivePowers(0,0,0,0);
        } else if (lower.contains("forward") || lower.matches(".*\\bfwd\\b.*")) {
            setDriveFromVectors(+p,0,0);
        } else if (lower.contains("back") || lower.contains("reverse") || lower.matches(".*\\brev\\b.*")) {
            setDriveFromVectors(-p,0,0);
        } else if (lower.contains("strafe right") || lower.matches(".*\\bsr\\b.*")) {
            setDriveFromVectors(0,+p,0);
        } else if (lower.contains("strafe left") || lower.matches(".*\\bsl\\b.*")) {
            setDriveFromVectors(0,-p,0);
        } else if (lower.contains("turn left") || lower.matches(".*\\btl\\b.*")) {
            setDriveFromVectors(0,0,+p);
        } else if (lower.contains("turn right") || lower.matches(".*\\btr\\b.*")) {
            setDriveFromVectors(0,0,-p);
        }
    }

    private String mailboxOrBusTake() {
        // 1) Bus queue (from subscription)
        String s = pendingCmds.poll();
        if (s != null) return s;
        // 2) JulesCommand mailbox (shared static)
        try {
            for (String m : new String[]{"getAndClear","take","poll","consume","next"}) {
                try { Method mm = JulesCommand.class.getMethod(m); Object out = mm.invoke(null); return out != null ? out.toString() : null; } catch (NoSuchMethodException ignored) {}
            }
            try {
                Method get = JulesCommand.class.getMethod("get");
                Object out = get.invoke(null);
                if (out == null) return null;
                try { JulesCommand.class.getMethod("clear").invoke(null);} catch (NoSuchMethodException e){ JulesCommand.class.getMethod("setCommand", String.class).invoke(null, (String) null);}
                return out.toString();
            } catch (NoSuchMethodException ignored) {}
        } catch (Throwable ignored) {}
        return null;
    }

    private void publishCmdStatus(String name, String status, JsonObject args) {
        JsonObject ev = new JsonObject();
        ev.addProperty("type", "cmd_status");
        ev.addProperty("name", name);
        ev.addProperty("status", status);
        ev.addProperty("ts_ms", System.currentTimeMillis());
        if (args != null) ev.add("args", args);
        busPublish(ev.toString());
    }

    private static String optString(JsonObject o, String k) {
        return (o != null && o.has(k) && o.get(k).isJsonPrimitive()) ? o.get(k).getAsString() : null;
    }

    private static double optDouble(JsonObject o, String[] keys, double def) {
        if (o == null) return def;
        for (String k : keys) {
            try { if (o.has(k) && o.get(k).isJsonPrimitive()) return o.get(k).getAsDouble(); } catch (Exception ignored) {}
        }
        return def;
    }

    private static double parseMagnitude(String s, double def) {
        try {
            String[] parts = s.replaceAll("[^0-9.+-]", " ").trim().split("\\s+");
            if (parts.length == 0 || parts[0].isEmpty()) return def;
            double v = Double.parseDouble(parts[0]);
            return Math.max(-1, Math.min(1, v));
        } catch (Exception e) { return def; }
    }

    // ------------------------------------------------------------
    // Drive helpers (mecanum)
    // ------------------------------------------------------------
    private void setDriveFromVectors(double fwd, double str, double trn) {
        double pLF = fwd + str + trn;
        double pRF = fwd - str - trn;
        double pLR = fwd - str + trn;
        double pRR = fwd + str - trn;
        double max = Math.max(1.0, Math.max(Math.abs(pLF), Math.max(Math.abs(pRF), Math.max(Math.abs(pLR), Math.abs(pRR)))));
        pLF /= max; pRF /= max; pLR /= max; pRR /= max;
        setDrivePowers(pLF, pRF, pLR, pRR);
    }

    private void setDrivePowers(double pLF, double pRF, double pLR, double pRR) {
        lf.setTarget(pLF); rf.setTarget(pRF); lr.setTarget(pLR); rr.setTarget(pRR);
    }

    private void stepKinematics(double dt) {
        double linScale = 40.0; // in/s per full power
        double rotScale = 90.0; // deg/s per full power diff
        double pLF = lf.power, pRF = rf.power, pLR = lr.power, pRR = rr.power;
        double vx = linScale * (pLF + pRF + pLR + pRR) / 4.0;                 // forward
        double vy = linScale * (-pLF + pRF + pLR - pRR) / 4.0;                // strafe
        double omega = rotScale * (-pLF + pRF - pLR + pRR) / 4.0;             // ccw
        xIn += vx * dt; yIn += vy * dt; headingDeg += omega * dt;
        while (headingDeg > 180) headingDeg -= 360; while (headingDeg < -180) headingDeg += 360;
    }

    private void updateBatteryModel(double dt) {
        double load = Math.abs(lf.power) + Math.abs(rf.power) + Math.abs(lr.power) + Math.abs(rr.power);
        double drop = load * BATTERY_LOAD_SAG_PER_SEC * dt;
        if (drop > 0) {
            batteryVoltage = Math.max(BATTERY_MIN_V, batteryVoltage - drop);
        }
        if (load < 0.25) {
            double recover = (0.25 - load) * BATTERY_RECOVERY_PER_SEC * dt;
            if (recover > 0) {
                batteryVoltage = Math.min(BATTERY_MAX_V, batteryVoltage + recover);
            }
        }
        dataOrganizer.updateBatteryOverride(batteryVoltage);
    }

    // ------------------------------------------------------------
    // Publishing
    // ------------------------------------------------------------
    private void publishHeartbeat() {
        JsonObject hb = new JsonObject();
        hb.addProperty("type", "heartbeat");
        hb.addProperty("ts_ms", System.currentTimeMillis());
        hb.addProperty("uptime_ms", (long) loopTimer.milliseconds());
        hb.addProperty("active_opmode", "JULES SimDrive v3");
        hb.addProperty("battery_v", batteryVoltage);
        hb.addProperty("port", bridgePort);
        busPublish(hb.toString());
    }

    private void publishSnapshot() {
        JsonObject snap = new JsonObject();
        snap.addProperty("type", "snapshot");
        snap.addProperty("ts_ms", System.currentTimeMillis());
        snap.addProperty("battery_v", batteryVoltage);

        JsonObject motors = new JsonObject();
        motors.add("lf", lf.toJson());
        motors.add("rf", rf.toJson());
        motors.add("lr", lr.toJson());
        motors.add("rr", rr.toJson());

        JsonObject pose = new JsonObject();
        pose.addProperty("x_in", xIn);
        pose.addProperty("y_in", yIn);
        pose.addProperty("heading_deg", headingDeg);

        JsonObject sim = new JsonObject();
        sim.add("motors", motors);
        sim.add("pose", pose);
        sim.addProperty("manual_mode", manualMode);
        sim.addProperty("battery_v", batteryVoltage);

        snap.add("sim", sim);
        busPublish(snap.toString());
    }

    private void busPublish(String json) {
        if (json == null) return;
        JulesStreamBus bus = resolveStreamBus();
        if (bus == null) return;
        try { bus.publishJsonLine(json); } catch (Throwable ignored) {}
    }

    private JulesStreamBus resolveStreamBus() {
        if (bridgeManager != null) {
            JulesStreamBus shared = bridgeManager.getStreamBus();
            if (shared != null) { streamBus = shared; return shared; }
        }
        return streamBus;
    }

    // ------------------------------------------------------------
    // Virtual motor model
    // ------------------------------------------------------------
    private static final class VirtualMotor {
        final String name;
        double target = 0;   // desired power [-1..1]
        double power = 0;    // current power [-1..1]
        double tau = 0.08;   // time constant
        VirtualMotor(String name) { this.name = name; }
        void setTarget(double t) { this.target = clamp(t); }
        void update(double dt) {
            double alpha = dt / Math.max(1e-3, tau);
            if (alpha > 1) alpha = 1;
            power += alpha * (target - power);
            power = clamp(power);
        }
        String debug() { return String.format(Locale.US, "tgt=%.2f pow=%.2f", target, power); }
        JsonObject toJson() {
            JsonObject o = new JsonObject();
            o.addProperty("name", name);
            o.addProperty("target", target);
            o.addProperty("power", power);
            return o;
        }
        static double clamp(double v) { return Math.max(-1, Math.min(1, v)); }
    }

    // ------------------------------------------------------------
    // Panels helpers (combined gamepad + telemetry)
    // ------------------------------------------------------------
    private Object getCombined(PanelsSide side) {
        try {
            Object mgr = (side == PanelsSide.FIRST) ? panelsMgr1 : panelsMgr2;
            if (mgr == null || asCombinedMethod == null) return null;
            Gamepad base = (side == PanelsSide.FIRST) ? gamepad1 : gamepad2;
            return asCombinedMethod.invoke(mgr, base);
        } catch (Throwable ignored) { return null; }
    }

    private boolean readButton(String name) {
        Object combined = getCombined(PanelsSide.FIRST);
        if (combined != null) {
            Boolean v = getBoolField(combined, name);
            if (v != null) return v;
        }
        return getRawButton(gamepad1, name);
    }

    private double readAxis(String name) {
        Object combined = getCombined(PanelsSide.FIRST);
        if (combined != null) {
            Double v = getDoubleField(combined, name);
            if (v != null) return v;
        }
        return getRawAxis(gamepad1, name);
    }

    private static Boolean getBoolField(Object o, String field) {
        try { Field f = o.getClass().getField(field); return f.getBoolean(o); } catch (Throwable ignored) { return null; }
    }
    private static Double getDoubleField(Object o, String field) {
        try { Field f = o.getClass().getField(field); return f.getDouble(o); } catch (Throwable ignored) { return null; }
    }

    private static boolean getRawButton(Gamepad gp, String name) {
        try {
            Field f = Gamepad.class.getField(name);
            if (f.getType() == boolean.class) return f.getBoolean(gp);
            if (name.equals("options")) return gp.start;
            if (name.equals("back"))    return gp.back;
            if (name.equals("guide"))   return gp.guide;
        } catch (Throwable ignored) {}
        return false;
    }

    private static double getRawAxis(Gamepad gp, String name) {
        try {
            Field f = Gamepad.class.getField(name);
            if (f.getType() == float.class) return f.getFloat(gp);
            if (f.getType() == double.class) return f.getDouble(gp);
        } catch (Throwable ignored) {}
        return 0.0;
    }

    private enum PanelsSide { FIRST, SECOND }

    // Panels Telemetry wrappers
    private void ptInfo(String msg) {
        if (panelsTelemetryObj == null) return;
        try { panelsTelemetryObj.getClass().getMethod("info", String.class).invoke(panelsTelemetryObj, msg); } catch (Throwable ignored) {}
    }
    private void ptDebug(String msg) {
        if (panelsTelemetryObj == null) return;
        try { panelsTelemetryObj.getClass().getMethod("debug", String.class).invoke(panelsTelemetryObj, msg); } catch (Throwable ignored) {}
    }
    private void ptUpdate() {
        if (panelsTelemetryObj == null) return;
        try { panelsTelemetryObj.getClass().getMethod("update", org.firstinspires.ftc.robotcore.external.Telemetry.class).invoke(panelsTelemetryObj, telemetry); } catch (Throwable ignored) {}
    }
}
