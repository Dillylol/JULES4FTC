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

// Optional: these imports resolve at compile time if the Panels libs are present
// We will still guard all usage with reflection so it won't crash if absent.
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.gamepad.PanelsGamepad;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Locale;

/**
 * JULESSimDrive v2 — virtual drivetrain with JULES hooks + structured command support
 * + Panels Gamepad input (combined with FTC gamepad).
 *
 * Responds to frames like:
 * {
 *   "label": "Drive Forward",
 *   "name": "drive",
 *   "args": { "t": 0.5, "p": 0.5, "duration_ms": 400 }
 * }
 * and also supports "strafe", "turn", and "stop".
 */
@TeleOp(name = "JULES SimDrive (Panels)", group = "JULES")
public class JULESSimDrive extends OpMode {

    // ---- JULES plumbing ----
    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;
    private int bridgePort = 58080;

    // ---- Sim state ----
    private final VirtualMotor lf = new VirtualMotor("lf");
    private final VirtualMotor rf = new VirtualMotor("rf");
    private final VirtualMotor lr = new VirtualMotor("lr");
    private final VirtualMotor rr = new VirtualMotor("rr");

    // simple pose (inches + degrees), purely illustrative
    private double xIn = 0, yIn = 0, headingDeg = 0;

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

    // ---- Panels integration (guarded with reflection) ----
    private Object panelsTelemetryObj;          // PanelsTelemetry.INSTANCE.getTelemetry()
    private Object panelsMgr1;                  // PanelsGamepad.INSTANCE.getFirstManager()
    private Object panelsMgr2;                  // PanelsGamepad.INSTANCE.getSecondManager()
    private Method asCombinedMethod;            // asCombinedFTCGamepad(Gamepad)

    @Override
    public void init() {
        // JULES streaming via shared bridge manager
        bridgeManager = JulesBridgeManager.getInstance();
        if (bridgeManager != null) {
            // Ensure the manager knows about our context; it will only auto-start if previously configured
            bridgeManager.prepare(hardwareMap.appContext);
            bridgePort = bridgeManager.getPort();
            streamBus = bridgeManager.getStreamBus();
        }

        // Panels telemetry + gamepad managers (use Kotlin object INSTANCE from Java)
        try {
            panelsTelemetryObj = PanelsTelemetry.INSTANCE.getTelemetry();
        } catch (Throwable t) {
            panelsTelemetryObj = null;
        }
        try {
            panelsMgr1 = PanelsGamepad.INSTANCE.getFirstManager();
            panelsMgr2 = PanelsGamepad.INSTANCE.getSecondManager();
            if (panelsMgr1 != null) {
                asCombinedMethod = panelsMgr1.getClass().getMethod("asCombinedFTCGamepad", Gamepad.class);
            }
        } catch (Throwable ignored) {
            panelsMgr1 = null;
            panelsMgr2 = null;
            asCombinedMethod = null;
        }

        ptInfo("JULES SimDrive (Panels) online");
        telemetry.addLine("JULES SimDrive (Panels) online");
        if (bridgeManager != null) {
            String adv = bridgeManager.getAdvertiseLine();
            telemetry.addLine(adv);
            ptInfo(adv);
            if (bridgeManager.getStreamBus() == null) {
                String offline = "JULES bridge offline - run 'JULES: Enable & Status'.";
                telemetry.addLine(offline);
                ptInfo(offline);
            }
        } else {
            String offline = "JULES bridge unavailable.";
            telemetry.addLine(offline);
            ptInfo(offline);
        }
        telemetry.update();

        loopTimer.reset();
    }

    @Override
    public void start() {
        publishHeartbeat();
        publishSnapshot();
    }

    @Override
    public void loop() {
        final double nowMs = loopTimer.milliseconds();
        final double dt = Math.max(0.001, (nowMs - lastSnapshotMs) / 1000.0);

        // 1) Handle structured / text commands
        handleIncomingCommand();

        // 1b) End timed commands
        if (cmdActive && nowMs >= cmdEndMs) {
            setDrivePowers(0,0,0,0);
            publishCmdStatus(cmdName, "completed", null);
            cmdActive = false;
            cmdName = null;
            manualMode = true; // fall back to manual after a timed command ends
        }

        // 2) Optional manual drive (Panels-combined preferred, FTC fallback)
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

        // 4) Toy kinematics
        stepKinematics(dt);

        // 5) Telemetry
        if (nowMs - lastHeartbeatMs >= 1000) { publishHeartbeat(); lastHeartbeatMs = nowMs; }
        if (nowMs - lastSnapshotMs  >= 100)  { publishSnapshot();  lastSnapshotMs  = nowMs; }

        String pose = String.format(Locale.US, "x=%.1f y=%.1f θ=%.1f°", xIn, yIn, headingDeg);
        telemetry.addData("mode", manualMode ? "manual" : "programmatic");
        telemetry.addData("cmdActive", cmdActive);
        telemetry.addData("pose", pose);
        telemetry.addData("lf", lf.debug());
        telemetry.addData("rf", rf.debug());
        telemetry.addData("lr", lr.debug());
        telemetry.addData("rr", rr.debug());
        telemetry.update();

        ptDebug("mode=" + (manualMode ? "manual" : "programmatic") + ", cmdActive=" + cmdActive);
        ptDebug("pose " + pose);
        ptDebug("lf " + lf.debug());
        ptDebug("rf " + rf.debug());
        ptDebug("lr " + lr.debug());
        ptDebug("rr " + rr.debug());
        ptUpdate();
    }

    @Override
    public void stop() {
    }

    // ------------------------------------------------------------
    // Command handling
    // ------------------------------------------------------------
    private void handleIncomingCommand() {
        try {
            String raw = mailboxTake();
            if (raw == null) return;
            raw = raw.trim();

            // ---------- Prefer structured JSON ----------
            if (raw.startsWith("{") && raw.endsWith("}")) {
                JsonElement el = GsonCompat.parse(raw);
                if (el != null && el.isJsonObject()) {
                    JsonObject root = el.getAsJsonObject();

                    // Unwrap {"type":"cmd","text": ...} where text may be OBJECT or STRING
                    if (root.has("type") && root.has("text")) {
                        JsonElement txt = root.get("text");
                        if (txt.isJsonObject()) {
                            root = txt.getAsJsonObject();
                        } else if (txt.isJsonPrimitive() && txt.getAsJsonPrimitive().isString()) {
                            JsonElement inner = GsonCompat.parse(txt.getAsString());
                            if (inner != null && inner.isJsonObject()) {
                                root = inner.getAsJsonObject();
                            } else {
                                handlePlainText(txt.getAsString().toLowerCase(Locale.US));
                                return;
                            }
                        }
                    }

                    // Now root is expected to look like:
                    // { "label": "...", "name": "drive|strafe|turn|stop", "args": { ... } }
                    String name = optString(root, "name");
                    String type = (name == null) ? optString(root, "type") : null; // also allow {"type":"drive", ...}

                    // Normalize which key we're using
                    String verb = (name != null) ? name : (type != null ? type : null);
                    if (verb != null) verb = verb.toLowerCase(Locale.US);

                    if ("drive".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject()
                                ? root.getAsJsonObject("args") : new JsonObject();

                        double p  = optDouble(args, new String[]{"p","power","fwd","forward"}, 0.0); // forward
                        double t  = optDouble(args, new String[]{"t","turn"}, 0.0);                 // turn
                        double s  = optDouble(args, new String[]{"s","strafe","x"}, 0.0);           // strafe
                        int ms    = (int)Math.round(optDouble(args, new String[]{"duration_ms","ms","duration"}, 0.0));

                        manualMode = false;
                        setDriveFromVectors(p, s, t);
                        publishCmdStatus("drive", "started", args);
                        if (ms > 0) {
                            cmdActive = true;
                            cmdEndMs  = loopTimer.milliseconds() + ms;
                            cmdName   = "drive";
                        }
                        return;
                    }

                    if ("strafe".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject()
                                ? root.getAsJsonObject("args") : new JsonObject();

                        double s = optDouble(args, new String[]{"speed","s","strafe","x"}, 0.0);
                        int ms   = (int)Math.round(optDouble(args, new String[]{"duration_ms","ms","duration"}, 0.0));

                        manualMode = false;
                        setDriveFromVectors(0, s, 0);
                        publishCmdStatus("strafe", "started", args);
                        if (ms > 0) {
                            cmdActive = true;
                            cmdEndMs  = loopTimer.milliseconds() + ms;
                            cmdName   = "strafe";
                        }
                        return;
                    }

                    if ("turn".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject()
                                ? root.getAsJsonObject("args") : new JsonObject();

                        double sp = optDouble(args, new String[]{"speed","t","turn"}, 0.0);
                        int ms    = (int)Math.round(optDouble(args, new String[]{"duration_ms","ms","duration"}, 0.0));

                        manualMode = false;
                        setDriveFromVectors(0, 0, sp);
                        publishCmdStatus("turn", "started", args);
                        if (ms > 0) {
                            cmdActive = true;
                            cmdEndMs  = loopTimer.milliseconds() + ms;
                            cmdName   = "turn";
                        }
                        return;
                    }

                    if ("stop".equals(verb)) {
                        manualMode = false;
                        setDrivePowers(0,0,0,0);
                        publishCmdStatus("stop", "completed", root);
                        cmdActive = false;
                        cmdName   = null;
                        return;
                    }

                    // Unknown JSON command → fall back to plain text if there is a "text" field
                    if (root.has("text") && root.get("text").isJsonPrimitive()) {
                        handlePlainText(root.get("text").getAsString().toLowerCase(Locale.US));
                        return;
                    }
                }
            }

            // ---------- Plain-text fallback (e.g., "forward 0.5 400") ----------
            handlePlainText(raw.toLowerCase(Locale.US));

        } catch (Throwable ignored) { }
    }

    // Plain-text helper (unchanged from your previous version)
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
            try {
                if (o.has(k) && o.get(k).isJsonPrimitive()) return o.get(k).getAsDouble();
            } catch (Exception ignored) {}
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
        double linScale = 40.0; // in/s
        double rotScale = 90.0; // deg/s
        double pLF = lf.power, pRF = rf.power, pLR = lr.power, pRR = rr.power;
        double vx = linScale * (pLF + pRF + pLR + pRR) / 4.0;                 // forward
        double vy = linScale * (-pLF + pRF + pLR - pRR) / 4.0;                // strafe
        double omega = rotScale * (-pLF + pRF - pLR + pRR) / 4.0;             // ccw
        xIn += vx * dt; yIn += vy * dt; headingDeg += omega * dt;
        while (headingDeg > 180) headingDeg -= 360; while (headingDeg < -180) headingDeg += 360;
    }

    // ------------------------------------------------------------
    // Publishing
    // ------------------------------------------------------------
    private void publishHeartbeat() {
        JsonObject hb = new JsonObject();
        hb.addProperty("type", "heartbeat");
        hb.addProperty("ts_ms", System.currentTimeMillis());
        hb.addProperty("uptime_ms", (long) loopTimer.milliseconds());
        hb.addProperty("active_opmode", "JULES SimDrive (Panels)");
        hb.addProperty("battery_v", 12.5);
        hb.addProperty("port", bridgePort);
        busPublish(hb.toString());
    }

    private void publishSnapshot() {
        JsonObject snap = new JsonObject();
        snap.addProperty("type", "snapshot");
        snap.addProperty("ts_ms", System.currentTimeMillis());

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

        snap.add("sim", sim);
        busPublish(snap.toString());
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
    // Compat helpers (JULES command mailbox + stream bus)
    // ------------------------------------------------------------
    private String mailboxTake() {
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

    private void busPublish(String json) {
        if (json == null) return;
        JulesStreamBus bus = resolveStreamBus();
        if (bus == null) return;
        try {
            bus.publishJsonLine(json);
        } catch (Throwable ignored) {}
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

    // ------------------------------------------------------------
    // Panels helpers (combined gamepad + telemetry via reflection)
    // ------------------------------------------------------------
    private Object getCombined(PanelsSide side) {
        try {
            Object mgr = (side == PanelsSide.FIRST) ? panelsMgr1 : panelsMgr2;
            if (mgr == null || asCombinedMethod == null) return null;
            Gamepad base = (side == PanelsSide.FIRST) ? gamepad1 : gamepad2;
            return asCombinedMethod.invoke(mgr, base);
        } catch (Throwable ignored) {
            return null;
        }
    }

    private boolean readButton(String name) {
        // prefer Panels-combined g1, fallback to raw FTC
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
            // synonyms
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

    // Panels Telemetry wrappers (avoid hard compile-time coupling)
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