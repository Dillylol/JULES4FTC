package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesCommand;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;

// Optional Panels libs (guarded)
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.gamepad.PanelsGamepad;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Locale;
import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

@TeleOp(name = "JULES CommandDrive", group = "JULES")
public class JULESCommandDrive extends OpMode {

    // ---- Hardware ----
    private DcMotor lf, rf, lr, rr;   // mapped via BjornHardware
    private IMU imu;

    // ---- JULES plumbing ----
    private JulesBridgeManager bridgeManager;
    private JulesStreamBus streamBus;

    // Bus subscription (SimDriver parity)
    private JulesStreamBus.Subscription busSub;
    private Thread busPump;
    private final Queue<String> pendingCmds = new ConcurrentLinkedQueue<>();

    // ---- Panels (optional) ----
    private Object panelsTelemetryObj;                   // PanelsTelemetry.INSTANCE.getTelemetry()
    private Object panelsMgr1;                           // PanelsGamepad.INSTANCE.getFirstManager()
    private Method asCombinedMethod;                     // asCombinedFTCGamepad(Gamepad)

    // ---- State ----
    private final ElapsedTime t = new ElapsedTime();
    private boolean manualMode = true;           // X toggles
    private boolean xPrev = false;

    private boolean cmdActive = false;
    private double cmdEndMs = 0;
    private String cmdName = null;
    private String lastCmdConsumed = "(none)";

    @Override public void init() {
        telemetry.setMsTransmissionInterval(50);

        // Map motors & sensors using your shared hardware wrapper
        BjornHardware hw = BjornHardware.forTeleOp(hardwareMap);
        lf = hw.frontLeft;  rf = hw.frontRight;  lr = hw.backLeft;  rr = hw.backRight;
        imu = hw.imu;

        // Match TeleOp hub orientation for yaw
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(params);

        // JULES bridge & bus
        bridgeManager = JulesBridgeManager.getInstance();
        if (bridgeManager != null) {
            bridgeManager.prepare(hardwareMap.appContext);
            streamBus = bridgeManager.getStreamBus();

            // Subscribe like SimDriver: collect {type:"cmd", text:...} frames OR raw strings
            if (streamBus != null) {
                try {
                    busSub = streamBus.subscribe();
                    busPump = new Thread(() -> {
                        try {
                            for (;;) {
                                String line = busSub.take();      // blocking until stop()
                                if (line == null) break;
                                JsonElement el = GsonCompat.parse(line);
                                if (el != null && el.isJsonObject()) {
                                    JsonObject o = el.getAsJsonObject();
                                    String typ = optString(o, "type");
                                    if (typ != null && typ.equalsIgnoreCase("cmd")) {
                                        if (o.has("text")) {
                                            if (o.get("text").isJsonObject()) {
                                                pendingCmds.offer(o.getAsJsonObject("text").toString());
                                            } else if (o.get("text").isJsonPrimitive()) {
                                                pendingCmds.offer(o.get("text").getAsString());
                                            }
                                        }
                                    }
                                } else {
                                    // non-JSON line: treat as plain text command
                                    pendingCmds.offer(line);
                                }
                            }
                        } catch (InterruptedException ignored) {
                            // exit quietly
                        } catch (Throwable t) {
                            // swallow to avoid crashing OpMode
                        }
                    }, "JULES-BusPump");
                    busPump.setDaemon(true);
                    busPump.start();
                } catch (Throwable ignored) {}
            }
        }

        // Panels singletons (optional)
        try { panelsTelemetryObj = PanelsTelemetry.INSTANCE.getTelemetry(); } catch (Throwable ignored) {}
        try {
            panelsMgr1 = PanelsGamepad.INSTANCE.getFirstManager();
            if (panelsMgr1 != null) asCombinedMethod = panelsMgr1.getClass().getMethod("asCombinedFTCGamepad", Gamepad.class);
        } catch (Throwable ignored) { panelsMgr1 = null; asCombinedMethod = null; }

        ptInfo("JULES CommandDrive online — X toggles MANUAL/PROGRAM");
        t.reset();
    }

    @Override public void start() {
        publishHeartbeat();
        publishSnapshot(0,0,0,0);
    }

    @Override public void loop() {
        final double nowMs = t.milliseconds();

        // 1) Consume a command if available
        handleIncomingCommand();

        // 1b) End timed command
        if (cmdActive && nowMs >= cmdEndMs) {
            setDrivePowers(0,0,0,0);
            publishCmdStatus(cmdName, "completed", null);
            cmdActive = false; cmdName = null; manualMode = true;
        }

        // 2) Manual override (Panels-combined if available)
        boolean x = readButton("x");
        if (x && !xPrev) manualMode = !manualMode;
        xPrev = x;

        if (manualMode) {
            double fwd = -readAxis("left_stick_y");
            double str =  readAxis("left_stick_x");
            double trn =  readAxis("right_stick_x");
            setDriveFromVectors(fwd, str, trn); // field-centric + normalized (matches TeleOp)
        }

        // 3) Periodic JULES streams (approx rates)
        if (((long)nowMs) % 1000 < 25) publishHeartbeat();
        if (((long)nowMs) % 100  < 25) publishSnapshot(lf.getPower(), rf.getPower(), lr.getPower(), rr.getPower());

        // 4) Driver Station telemetry
        telemetry.addData("mode", manualMode ? "MANUAL" : "PROGRAM");
        telemetry.addData("lastCmd", lastCmdConsumed);
        telemetry.addData("lf/rf/lr/rr", "%.2f %.2f %.2f %.2f", lf.getPower(), rf.getPower(), lr.getPower(), rr.getPower());
        telemetry.update();
        ptUpdate();
    }

    @Override public void stop() {
        setDrivePowers(0,0,0,0);
        // Cleanly stop bus pump
        try { if (busSub != null) busSub.close(); } catch (Throwable ignored) {}
        try { if (busPump != null) busPump.interrupt(); } catch (Throwable ignored) {}
    }

    // ------------------------------------------------------------
    // Command handling (JSON or plain text) — SimDriver-compatible
    // ------------------------------------------------------------
    private void handleIncomingCommand() {
        try {
            String raw = mailboxOrBusTake();
            if (raw == null) return;
            raw = raw.trim();

            // Prefer JSON
            if (raw.startsWith("{") && raw.endsWith("}")) {
                JsonElement el = GsonCompat.parse(raw);
                if (el != null && el.isJsonObject()) {
                    JsonObject root = el.getAsJsonObject();

                    // Accept frames like {type:"cmd", text:<OBJECT|STRING>}
                    if (root.has("type") && root.has("text")) {
                        JsonElement txt = root.get("text");
                        if (txt.isJsonObject()) root = txt.getAsJsonObject();
                        else if (txt.isJsonPrimitive()) {
                            JsonElement inner = GsonCompat.parse(txt.getAsString());
                            if (inner != null && inner.isJsonObject()) root = inner.getAsJsonObject();
                            else { handlePlainText(txt.getAsString().toLowerCase(Locale.US)); lastCmdConsumed = txt.getAsString(); return; }
                        }
                    }

                    // Accept {name:"drive", args:{...}} OR {type:"drive", args:{...}}
                    String name = optString(root, "name");
                    String type = (name == null) ? optString(root, "type") : null;
                    String verb = (name != null ? name : type);
                    if (verb != null) verb = verb.toLowerCase(Locale.US);

                    if ("drive".equals(verb) || "move".equals(verb) || "go".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject() ? root.getAsJsonObject("args") : new JsonObject();
                        double p  = optDouble(args, new String[]{"p","power","fwd","forward","y"}, 0.0);
                        double t  = optDouble(args, new String[]{"t","turn","omega","w","rz"}, 0.0);
                        double s  = optDouble(args, new String[]{"s","strafe","x"}, 0.0);
                        int ms    = parseDurationFields(args, 0);
                        manualMode = false; setDriveFromVectors(p, s, t); publishCmdStatus("drive","started", args);
                        if (ms > 0) { cmdActive = true; cmdEndMs = this.t.milliseconds() + ms; cmdName = "drive"; }
                        lastCmdConsumed = root.toString(); return;
                    }
                    if ("strafe".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject() ? root.getAsJsonObject("args") : new JsonObject();
                        double s = optDouble(args, new String[]{"speed","s","strafe","x"}, 0.0);
                        int ms   = parseDurationFields(args, 0);
                        manualMode = false; setDriveFromVectors(0, s, 0); publishCmdStatus("strafe","started", args);
                        if (ms > 0) { cmdActive = true; cmdEndMs = this.t.milliseconds() + ms; cmdName = "strafe"; }
                        lastCmdConsumed = root.toString(); return;
                    }
                    if ("turn".equals(verb) || "rotate".equals(verb)) {
                        JsonObject args = root.has("args") && root.get("args").isJsonObject() ? root.getAsJsonObject("args") : new JsonObject();
                        double sp = optDouble(args, new String[]{"speed","t","turn","omega","w","rz"}, 0.0);
                        int ms    = parseDurationFields(args, 0);
                        manualMode = false; setDriveFromVectors(0, 0, sp); publishCmdStatus("turn","started", args);
                        if (ms > 0) { cmdActive = true; cmdEndMs = this.t.milliseconds() + ms; cmdName = "turn"; }
                        lastCmdConsumed = root.toString(); return;
                    }
                    if ("stop".equals(verb) || "halt".equals(verb)) {
                        manualMode = false; setDrivePowers(0,0,0,0); publishCmdStatus("stop","completed", root);
                        cmdActive = false; cmdName = null; lastCmdConsumed = root.toString(); return;
                    }

                    // Unknown JSON → try a text field
                    if (root.has("text") && root.get("text").isJsonPrimitive()) {
                        String s = root.get("text").getAsString();
                        handlePlainText(s.toLowerCase(Locale.US)); lastCmdConsumed = s; return;
                    }
                }
            }

            // Plain text fallback
            handlePlainText(raw.toLowerCase(Locale.US));
            lastCmdConsumed = raw;
        } catch (Throwable ignored) {}
    }

    private void handlePlainText(String lower) {
        manualMode = false;
        if (lower.contains("stop") || lower.contains("halt")) { setDrivePowers(0,0,0,0); return; }

        double p = parseMagnitude(lower, 0.6);
        int ms   = parseDurationMsFromText(lower, 0);

        if (lower.contains("forward") || lower.matches(".*\\bfwd\\b.*")) {
            beginTimedVec(+p, 0, 0, ms, "drive");
        } else if (lower.contains("back") || lower.contains("reverse") || lower.matches(".*\\brev\\b.*")) {
            beginTimedVec(-p, 0, 0, ms, "drive");
        } else if (lower.contains("strafe right") || lower.matches(".*\\bsr\\b.*")) {
            beginTimedVec(0, +p, 0, ms, "strafe");
        } else if (lower.contains("strafe left") || lower.matches(".*\\bsl\\b.*")) {
            beginTimedVec(0, -p, 0, ms, "strafe");
        } else if (lower.contains("turn left") || lower.matches(".*\\btl\\b.*")) {
            beginTimedVec(0, 0, +p, ms, "turn");
        } else if (lower.contains("turn right") || lower.matches(".*\\btr\\b.*")) {
            beginTimedVec(0, 0, -p, ms, "turn");
        }
    }

    private void beginTimedVec(double fwd, double str, double trn, int ms, String label) {
        setDriveFromVectors(fwd, str, trn);
        publishCmdStatus(label, "started", null);
        if (ms > 0) { cmdActive = true; cmdEndMs = t.milliseconds() + ms; cmdName = label; }
    }

    private String mailboxOrBusTake() {
        // 1) Bus queue (from subscription)
        String s = pendingCmds.poll();
        if (s != null) return s;

        // 2) JulesCommand mailbox (shared static), try several common method names
        try {
            for (String m : new String[]{"getAndClear","take","poll","consume","next"}) {
                try {
                    Method mm = JulesCommand.class.getMethod(m);
                    Object out = mm.invoke(null);
                    return out != null ? out.toString() : null;
                } catch (NoSuchMethodException ignored) {}
            }
            try {
                Method get = JulesCommand.class.getMethod("get");
                Object out = get.invoke(null);
                if (out == null) return null;
                try { JulesCommand.class.getMethod("clear").invoke(null); }
                catch (NoSuchMethodException e) { JulesCommand.class.getMethod("setCommand", String.class).invoke(null, (String) null); }
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

    // ------------------------------------------------------------
    // Drive helpers — EXACT drive math from TeleOp
    // ------------------------------------------------------------
    private void setDriveFromVectors(double fwd, double str, double trn) {
        // Field-centric transform using IMU yaw (radians)
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX =  str * Math.cos(-heading) - fwd * Math.sin(-heading);
        double rotY =  str * Math.sin(-heading) + fwd * Math.cos(-heading);
        double denom = Math.max(1.0, Math.abs(rotY) + Math.abs(rotX) + Math.abs(trn));
        double pLF = (rotY + rotX + trn) / denom;
        double pLR = (rotY - rotX + trn) / denom;
        double pRF = (rotY - rotX - trn) / denom;
        double pRR = (rotY + rotX - trn) / denom;
        setDrivePowers(pLF, pRF, pLR, pRR);
    }

    private void setDrivePowers(double pLF, double pRF, double pLR, double pRR) {
        lf.setPower(pLF); rf.setPower(pRF); lr.setPower(pLR); rr.setPower(pRR);
    }

    // ------------------------------------------------------------
    // JULES stream helpers
    // ------------------------------------------------------------
    private void publishHeartbeat() {
        JsonObject hb = new JsonObject();
        hb.addProperty("type", "heartbeat");
        hb.addProperty("ts_ms", System.currentTimeMillis());
        hb.addProperty("active_opmode", "JULES CommandDrive");
        busPublish(hb.toString());
    }

    private void publishSnapshot(double pLF, double pRF, double pLR, double pRR) {
        JsonObject snap = new JsonObject();
        snap.addProperty("type", "snapshot");
        snap.addProperty("ts_ms", System.currentTimeMillis());
        JsonObject powers = new JsonObject();
        powers.addProperty("lf", pLF);
        powers.addProperty("rf", pRF);
        powers.addProperty("lr", pLR);
        powers.addProperty("rr", pRR);
        snap.add("powers", powers);
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
    // Utils
    // ------------------------------------------------------------
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

    private static int parseDurationFields(JsonObject args, int defMs) {
        if (args == null) return defMs;
        try {
            // duration can be in ms or seconds; accept strings or numbers
            if (args.has("duration_ms")) return (int)Math.round(args.get("duration_ms").getAsDouble());
            if (args.has("ms"))         return (int)Math.round(args.get("ms").getAsDouble());
            if (args.has("duration")) {
                JsonElement e = args.get("duration");
                if (e.isJsonPrimitive()) {
                    String s = e.getAsString().trim().toLowerCase(Locale.US);
                    if (s.endsWith("ms")) return Integer.parseInt(s.replace("ms","").trim());
                    if (s.endsWith("s"))  return (int)Math.round(Double.parseDouble(s.replace("s","").trim()) * 1000.0);
                    // plain number → interpret as ms
                    return (int)Math.round(Double.parseDouble(s));
                } else {
                    return (int)Math.round(e.getAsDouble());
                }
            }
        } catch (Throwable ignored) {}
        return defMs;
    }

    private static double parseMagnitude(String s, double def) {
        try {
            String[] parts = s.replaceAll("[^0-9.+-]", " ").trim().split("\\s+");
            if (parts.length == 0 || parts[0].isEmpty()) return def;
            double v = Double.parseDouble(parts[0]);
            return Math.max(-1, Math.min(1, v));
        } catch (Exception e) { return def; }
    }

    private static int parseDurationMsFromText(String s, int def) {
        try {
            // accept "800", "0.8s", "800ms" as second number, if present
            String[] nums = s.toLowerCase(Locale.US).replaceAll("[^0-9.ms]", " ").trim().split("\\s+");
            if (nums.length < 2) return def;
            String tok = nums[1];
            if (tok.endsWith("ms")) return Integer.parseInt(tok.replace("ms",""));
            if (tok.endsWith("s"))  return (int)Math.round(Double.parseDouble(tok.replace("s","")) * 1000.0);
            return Integer.parseInt(tok);
        } catch (Exception e) { return def; }
    }

    // Panels helpers (optional)
    private Object getCombined() {
        try {
            if (panelsMgr1 == null || asCombinedMethod == null) return null;
            return asCombinedMethod.invoke(panelsMgr1, gamepad1);
        } catch (Throwable ignored) { return null; }
    }
    private boolean readButton(String name) {
        Object combined = getCombined();
        if (combined != null) {
            Boolean v = getBoolField(combined, name);
            if (v != null) return v;
        }
        return getRawButton(gamepad1, name);
    }
    private double readAxis(String name) {
        Object combined = getCombined();
        if (combined != null) {
            Double v = getDoubleField(combined, name);
            if (v != null) return v;
        }
        return getRawAxis(gamepad1, name);
    }
    private static Boolean getBoolField(Object o, String field) { try { Field f = o.getClass().getField(field); return f.getBoolean(o); } catch (Throwable ignored) { return null; } }
    private static Double  getDoubleField(Object o, String field){ try { Field f = o.getClass().getField(field); return f.getDouble(o);} catch (Throwable ignored) { return null; } }
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

    // Panels Telemetry wrappers
    private void ptInfo(String msg) { if (panelsTelemetryObj==null) return; try { panelsTelemetryObj.getClass().getMethod("info", String.class).invoke(panelsTelemetryObj, msg); } catch (Throwable ignored) {} }
    private void ptUpdate(){ if (panelsTelemetryObj==null) return; try { panelsTelemetryObj.getClass().getMethod("update", org.firstinspires.ftc.robotcore.external.Telemetry.class).invoke(panelsTelemetryObj, telemetry); } catch (Throwable ignored) {} }
}
