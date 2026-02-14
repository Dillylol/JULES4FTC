package org.firstinspires.ftc.teamcode.jules.shot;

import android.content.Context;

import androidx.annotation.Nullable;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;

import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.Locale;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;

/**
 * Bridge helper that hides the transport-layer specifics for the shot planner protocol.
 */
public final class ShotPlannerBridge implements AutoCloseable {

    public static final class PoseSnapshot {
        public final double xIn;
        public final double yIn;
        public final double headingDeg;

        public PoseSnapshot(double xIn, double yIn, double headingDeg) {
            this.xIn = xIn;
            this.yIn = yIn;
            this.headingDeg = headingDeg;
        }
    }

    public static final class ShotContext {
        public final PoseSnapshot pose;
        public final double rangeIn;
        public final double vBattLoad;
        public final double rpmMeasured;
        /** Nominal RPM before planner bias (already includes model deltas/session bias). */
        public final double rpmBase;
        public final double rpmTargetCmd;
        public final double headingToTagDeg;
        public final long timestampMs;

        public ShotContext(PoseSnapshot pose,
                           double rangeIn,
                           double vBattLoad,
                           double rpmMeasured,
                           double rpmBase,
                           double rpmTargetCmd,
                           double headingToTagDeg,
                           long timestampMs) {
            this.pose = pose;
            this.rangeIn = rangeIn;
            this.vBattLoad = vBattLoad;
            this.rpmMeasured = rpmMeasured;
            this.rpmBase = rpmBase;
            this.rpmTargetCmd = rpmTargetCmd;
            this.headingToTagDeg = headingToTagDeg;
            this.timestampMs = timestampMs;
        }
    }

    public static final class ShotToken {
        public final String trialId;
        public final long timestampMs;
        public final double rangeIn;
        public final double vBattLoad;
        public final double rpmTargetCmd;
        public final double rpmAtFire;
        public final long timeToReadyMs;
        public final PoseSnapshot pose;
        public final double headingToTagDeg;

        public ShotToken(String trialId,
                         long timestampMs,
                         double rangeIn,
                         double vBattLoad,
                         double rpmTargetCmd,
                         double rpmAtFire,
                         long timeToReadyMs,
                         PoseSnapshot pose,
                         double headingToTagDeg) {
            this.trialId = trialId;
            this.timestampMs = timestampMs;
            this.rangeIn = rangeIn;
            this.vBattLoad = vBattLoad;
            this.rpmTargetCmd = rpmTargetCmd;
            this.rpmAtFire = rpmAtFire;
            this.timeToReadyMs = timeToReadyMs;
            this.pose = pose;
            this.headingToTagDeg = headingToTagDeg;
        }
    }

    public static final class ShotResult {
        public final String trialId;
        public final boolean hit;
        public final long latencyMs;

        ShotResult(String trialId, boolean hit, long latencyMs) {
            this.trialId = trialId;
            this.hit = hit;
            this.latencyMs = latencyMs;
        }
    }

    public static final class HelloAck {
        public final String sessionId;
        public final int modelVersion;
        @Nullable
        public final JsonObject policy;

        HelloAck(String sessionId, int modelVersion, @Nullable JsonObject policy) {
            this.sessionId = sessionId;
            this.modelVersion = modelVersion;
            this.policy = policy;
        }
    }

    private static final String TAG = "ShotPlannerBridge";
    private static final int MAX_QUEUE = 256;
    private static final int MAX_SEEN_CMDS = 64;
    private static final int MAX_SEEN_UPDATES = 32;

    private final OkHttpClient client;
    private final ScheduledExecutorService executor;
    private final Queue<String> outboundQueue = new ConcurrentLinkedQueue<>();
    private final ArrayDeque<JsonObject> inboundQueue = new ArrayDeque<>();
    private final ArrayDeque<ShotResult> shotResults = new ArrayDeque<>();
    private final ArrayDeque<HelloAck> helloAcks = new ArrayDeque<>();
    private final Object inboundLock = new Object();
    private final Object resultLock = new Object();
    private final Object helloLock = new Object();
    private final ArrayDeque<String> recentCmdIds = new ArrayDeque<>();
    private final Set<String> recentCmdSet = new HashSet<>();
    private final ArrayDeque<String> recentUpdateIds = new ArrayDeque<>();
    private final Set<String> recentUpdateSet = new HashSet<>();
    private final Gson gson = new Gson();
    private final String wsUrl;

    private final AtomicBoolean running = new AtomicBoolean(false);
    private final AtomicBoolean connected = new AtomicBoolean(false);

    private WebSocket socket;
    private int reconnectAttempts = 0;

    private String sessionId = "";
    private String botId = "";
    private long sessionEpochMs = 0L;

    private long obsSeq = 0L;
    private long requestSeq = 0L;
    private long tokenSeq = 0L;

    public ShotPlannerBridge(Context context) {
        String ip = ShotTrainerSettings.getClientIp(context);
        int port = ShotTrainerSettings.getClientPort(context);
        this.wsUrl = String.format(Locale.US, "ws://%s:%d/jules", ip, port);
        this.client = new OkHttpClient.Builder()
                .readTimeout(0, TimeUnit.MILLISECONDS)
                .build();
        this.executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "ShotPlannerBridge");
            t.setDaemon(true);
            return t;
        });
    }

    public void setSessionId(String sessionId) {
        this.sessionId = sessionId != null ? sessionId : "";
    }

    public void setBotId(String botId) {
        this.botId = botId != null ? botId : "";
    }

    public void setSessionEpochMs(long epochMs) {
        this.sessionEpochMs = epochMs;
    }

    public String getEndpoint() {
        return wsUrl;
    }

    public boolean isConnected() {
        return connected.get();
    }

    public void connect() {
        if (running.compareAndSet(false, true)) {
            scheduleConnect(0L);
        }
    }

    private void scheduleConnect(long delayMs) {
        executor.schedule(this::openSocket, delayMs, TimeUnit.MILLISECONDS);
    }

    private void openSocket() {
        if (!running.get()) {
            return;
        }
        Request request = new Request.Builder().url(wsUrl).build();
        socket = client.newWebSocket(request, new WebSocketListener() {
            @Override
            public void onOpen(WebSocket webSocket, Response response) {
                connected.set(true);
                reconnectAttempts = 0;
                RobotLog.ii(TAG, "Jules connected (%s)", wsUrl);
                flushOutbound();
                sendHelloOnConnect();
            }

            @Override
            public void onMessage(WebSocket webSocket, String text) {
                handleInboundText(text);
            }

            @Override
            public void onClosed(WebSocket webSocket, int code, String reason) {
                connected.set(false);
                RobotLog.w(TAG, "Jules link closed (%d %s)", code, reason);
                scheduleReconnect();
            }

            @Override
            public void onFailure(WebSocket webSocket, Throwable t, Response response) {
                connected.set(false);
                RobotLog.ee(TAG, t, "WebSocket failure: %s", t.getMessage());
                scheduleReconnect();
            }
        });
    }

    private void scheduleReconnect() {
        if (!running.get()) {
            return;
        }
        long delay = Math.min(5_000L, (long) Math.pow(2.0, reconnectAttempts) * 250L);
        reconnectAttempts = Math.min(reconnectAttempts + 1, 8);
        scheduleConnect(delay);
    }

    private void flushOutbound() {
        if (!connected.get()) {
            return;
        }
        String next;
        while ((next = outboundQueue.poll()) != null) {
            WebSocket current = socket;
            if (current != null) {
                current.send(next);
            }
        }
    }

    private void enqueue(JsonObject payload) {
        if (payload == null) {
            return;
        }
        String json = gson.toJson(payload);
        if (!json.endsWith("\n")) {
            json = json + "\n";
        }
        if (outboundQueue.size() > MAX_QUEUE) {
            outboundQueue.poll();
        }
        outboundQueue.offer(json);
        flushOutbound();
    }

    private void handleInboundText(String text) {
        if (text == null || text.isEmpty()) {
            return;
        }
        String[] lines = text.split("\r?\n");
        for (String line : lines) {
            String trimmed = line.trim();
            if (trimmed.isEmpty()) {
                continue;
            }
            JsonElement parsed = GsonCompat.parse(trimmed);
            if (parsed == null || !parsed.isJsonObject()) {
                continue;
            }
            JsonObject obj = parsed.getAsJsonObject();
            String type = optString(obj, "type");
            if ("hello_ack".equalsIgnoreCase(type)) {
                HelloAck ack = parseHelloAck(obj);
                if (ack != null) {
                    synchronized (helloLock) {
                        helloAcks.addLast(ack);
                    }
                }
                continue;
            }
            if ("shot_result".equalsIgnoreCase(type)) {
                ShotResult result = parseShotResult(obj);
                if (result != null) {
                    synchronized (resultLock) {
                        shotResults.addLast(result);
                    }
                }
                continue;
            }
            if ("cmd".equalsIgnoreCase(type)
                    || "rpm_model_update".equalsIgnoreCase(type)
                    || "shot_plan".equalsIgnoreCase(type)) {
                synchronized (inboundLock) {
                    inboundQueue.addLast(obj);
                    while (inboundQueue.size() > MAX_QUEUE) {
                        inboundQueue.removeFirst();
                    }
                }
                continue;
            }
            RobotLog.v(TAG, "Ignoring inbound frame type=%s", type);
        }
    }

    @Nullable
    public JsonObject pollInbound() {
        synchronized (inboundLock) {
            return inboundQueue.pollFirst();
        }
    }

    @Nullable
    public ShotResult pollShotResult() {
        synchronized (resultLock) {
            return shotResults.pollFirst();
        }
    }

    @Nullable
    public HelloAck pollHelloAck() {
        synchronized (helloLock) {
            return helloAcks.pollFirst();
        }
    }

    public void sendHello(String sessionId, long nowMs) {
        setSessionId(sessionId);
        enqueue(buildHelloPayload(nowMs));
    }

    public void sendObservation(ShotContext context) {
        if (context == null || context.pose == null) {
            return;
        }
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "obs");
        payload.addProperty("session_id", sessionId);
        payload.addProperty("seq", ++obsSeq);
        payload.addProperty("ts", context.timestampMs);
        payload.add("pose", buildPose(context.pose));
        payload.addProperty("range_in", context.rangeIn);
        payload.addProperty("v_batt_load", context.vBattLoad);
        payload.addProperty("rpm_meas", context.rpmMeasured);
        payload.addProperty("rpm_base", context.rpmBase);
        payload.addProperty("rpm_tgt_cmd", context.rpmTargetCmd);
        payload.addProperty("heading_to_tag_deg", context.headingToTagDeg);
        enqueue(payload);
    }

    public void sendRequestShotPlan(ShotContext context) {
        if (context == null || context.pose == null) {
            return;
        }
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "request_shot_plan");
        payload.addProperty("session_id", sessionId);
        payload.addProperty("seq", ++requestSeq);
        payload.addProperty("ts", context.timestampMs);
        payload.add("pose", buildPose(context.pose));
        payload.addProperty("range_in", context.rangeIn);
        payload.addProperty("v_batt_load", context.vBattLoad);
        payload.addProperty("rpm_meas", context.rpmMeasured);
        payload.addProperty("rpm_base", context.rpmBase);
        payload.addProperty("rpm_tgt_cmd", context.rpmTargetCmd);
        payload.addProperty("heading_to_tag_deg", context.headingToTagDeg);
        enqueue(payload);
    }

    public void sendShotToken(ShotToken token) {
        if (token == null || token.pose == null) {
            return;
        }
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "token_shot_fired");
        payload.addProperty("session_id", sessionId);
        payload.addProperty("seq", ++tokenSeq);
        payload.addProperty("trial_id", token.trialId);
        payload.addProperty("ts", token.timestampMs);
        payload.addProperty("range_in", token.rangeIn);
        payload.addProperty("v_batt_load", token.vBattLoad);
        payload.addProperty("rpm_tgt_cmd", token.rpmTargetCmd);
        payload.addProperty("rpm_at_fire", token.rpmAtFire);
        payload.addProperty("t_to_ready_ms", token.timeToReadyMs);
        payload.add("pose", buildPose(token.pose));
        payload.addProperty("heading_to_tag_deg", token.headingToTagDeg);
        enqueue(payload);
    }

    private JsonObject buildPose(PoseSnapshot pose) {
        JsonObject obj = new JsonObject();
        obj.addProperty("x", pose.xIn);
        obj.addProperty("y", pose.yIn);
        obj.addProperty("h", pose.headingDeg);
        return obj;
    }

    private JsonObject buildHelloPayload(long nowMs) {
        JsonObject payload = new JsonObject();
        payload.addProperty("type", "hello");
        payload.addProperty("session_id", sessionId);
        payload.addProperty("bot_id", botId);
        payload.addProperty("sdk", "ftc");
        payload.addProperty("proto", "jcp.v2");
        payload.addProperty("ts", nowMs);
        return payload;
    }

    private void sendHelloOnConnect() {
        if (sessionId == null || sessionId.isEmpty()) {
            return;
        }
        enqueue(buildHelloPayload(System.currentTimeMillis()));
    }

    /** Attempt to parse a planner command; returns {@code null} when ignored. */
    @Nullable
    public ShotPlan tryParseShotPlan(JsonObject payload, long wallClockNowMs) {
        if (payload == null) {
            return null;
        }
        String type = optString(payload, "type");
        if (type != null && !"cmd".equalsIgnoreCase(type) && !"shot_plan".equalsIgnoreCase(type)) {
            return null;
        }

        String cmdId = optString(payload, "cmd_id");
        if (cmdId == null || cmdId.isEmpty()) {
            RobotLog.w(TAG, "Dropping planner command without cmd_id");
            return null;
        }
        if (recentCmdSet.contains(cmdId)) {
            RobotLog.v(TAG, "Ignoring duplicate planner command %s", cmdId);
            return null;
        }

        long validMs = optLong(payload, "valid_ms", -1L);
        long sentMs = optLong(payload, "sent_ms", optLong(payload, "ts", wallClockNowMs));
        if (validMs <= 0L) {
            RobotLog.w(TAG, "Dropping command %s missing TTL", cmdId);
            return null;
        }
        long validUntil = sentMs + validMs;
        if (wallClockNowMs > validUntil) {
            RobotLog.v(TAG, "Command %s expired (now=%d > %d)", cmdId, wallClockNowMs, validUntil);
            return null;
        }

        Double rpmBias = optNullableDouble(payload, "rpm_bias");
        Double rpmAbs = optNullableDouble(payload, "rpm_target_abs");
        if ((rpmBias == null && rpmAbs == null) || (rpmBias != null && rpmAbs != null)) {
            RobotLog.w(TAG, "Command %s missing or conflicting RPM directive", cmdId);
            return null;
        }

        double rangeDeltaIn = optDouble(payload, "range_delta_in", 0.0);
        boolean loiter = optBoolean(payload, "loiter", false);
        double headingOffset = optDouble(payload, "heading_offset_deg", 0.0);
        String trialId = optString(payload, "trial_id");
        if (trialId == null || trialId.isEmpty()) {
            trialId = cmdId;
        }

        rememberCommand(cmdId);
        return new ShotPlan(cmdId,
                trialId,
                rpmBias,
                rpmAbs,
                rangeDeltaIn,
                loiter,
                headingOffset,
                sentMs,
                validUntil,
                wallClockNowMs);
    }

    public boolean isModelUpdate(JsonObject payload) {
        if (payload == null) {
            return false;
        }
        String type = optString(payload, "type");
        if (type == null || !"rpm_model_update".equalsIgnoreCase(type)) {
            return false;
        }
        String updateId = optString(payload, "update_id");
        if (updateId != null && recentUpdateSet.contains(updateId)) {
            RobotLog.v(TAG, "Ignoring duplicate rpm_model_update %s", updateId);
            return false;
        }
        if (updateId != null) {
            rememberUpdate(updateId);
        }
        return true;
    }

    public boolean isShotResult(JsonObject payload) {
        if (payload == null) {
            return false;
        }
        String type = optString(payload, "type");
        return type != null && "shot_result".equalsIgnoreCase(type);
    }

    public void pushRawCommand(String raw) {
        if (raw == null || raw.isEmpty()) {
            return;
        }
        JsonObject obj = unwrap(raw);
        if (obj != null) {
            synchronized (inboundLock) {
                inboundQueue.addLast(obj);
                while (inboundQueue.size() > MAX_QUEUE) {
                    inboundQueue.removeFirst();
                }
            }
        }
    }

    private void rememberCommand(String cmdId) {
        if (cmdId == null) {
            return;
        }
        if (recentCmdSet.add(cmdId)) {
            recentCmdIds.addLast(cmdId);
            while (recentCmdIds.size() > MAX_SEEN_CMDS) {
                String evicted = recentCmdIds.removeFirst();
                recentCmdSet.remove(evicted);
            }
        }
    }

    private void rememberUpdate(String updateId) {
        if (updateId == null) {
            return;
        }
        if (recentUpdateSet.add(updateId)) {
            recentUpdateIds.addLast(updateId);
            while (recentUpdateIds.size() > MAX_SEEN_UPDATES) {
                String evicted = recentUpdateIds.removeFirst();
                recentUpdateSet.remove(evicted);
            }
        }
    }

    private HelloAck parseHelloAck(JsonObject obj) {
        String session = optString(obj, "session_id");
        int modelVersion = (int) optLong(obj, "model_version", -1);
        JsonObject policy = null;
        if (obj.has("policy") && obj.get("policy").isJsonObject()) {
            policy = obj.getAsJsonObject("policy");
        }
        return new HelloAck(session != null ? session : sessionId, modelVersion, policy);
    }

    private ShotResult parseShotResult(JsonObject obj) {
        String trial = optString(obj, "trial_id");
        if (trial == null || trial.isEmpty()) {
            trial = "unknown";
        }
        boolean hit = optBoolean(obj, "hit", false);
        long latency = optLong(obj, "latency_ms", -1L);
        return new ShotResult(trial, hit, latency);
    }

    /** Parse a raw command string into a JSON object, handling mailbox wrappers. */
    @Nullable
    public static JsonObject unwrap(String raw) {
        if (raw == null || raw.isEmpty()) {
            return null;
        }
        JsonElement element = GsonCompat.parse(raw);
        if (element == null || !element.isJsonObject()) {
            return null;
        }
        JsonObject obj = element.getAsJsonObject();
        String type = optString(obj, "type");
        if ("cmd".equalsIgnoreCase(type) && obj.has("text")) {
            JsonElement textElement = obj.get("text");
            if (textElement.isJsonObject()) {
                return textElement.getAsJsonObject();
            }
            if (textElement.isJsonPrimitive()) {
                JsonElement parsed = GsonCompat.parse(textElement.getAsString());
                if (parsed != null && parsed.isJsonObject()) {
                    return parsed.getAsJsonObject();
                }
            }
            return null;
        }
        return obj;
    }

    private static String optString(JsonObject obj, String field) {
        if (obj == null || field == null) {
            return null;
        }
        try {
            JsonElement el = obj.get(field);
            if (el == null || el.isJsonNull()) {
                return null;
            }
            if (el.isJsonPrimitive()) {
                return el.getAsString();
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    private static boolean optBoolean(JsonObject obj, String field, boolean def) {
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return def;
            }
            if (el.isJsonPrimitive()) {
                if (el.getAsJsonPrimitive().isBoolean()) {
                    return el.getAsBoolean();
                }
                if (el.getAsJsonPrimitive().isNumber()) {
                    return el.getAsInt() != 0;
                }
                if (el.getAsJsonPrimitive().isString()) {
                    String s = el.getAsString();
                    if ("true".equalsIgnoreCase(s) || "1".equals(s)) {
                        return true;
                    }
                    if ("false".equalsIgnoreCase(s) || "0".equals(s)) {
                        return false;
                    }
                }
            }
        } catch (Exception ignored) {
        }
        return def;
    }

    private static double optDouble(JsonObject obj, String field, double def) {
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return def;
            }
            if (el.isJsonPrimitive()) {
                return el.getAsDouble();
            }
        } catch (Exception e) {
            try {
                return Double.parseDouble(obj.get(field).getAsString());
            } catch (Exception ignored) {
            }
        }
        return def;
    }

    private static Double optNullableDouble(JsonObject obj, String field) {
        if (obj == null || field == null || !obj.has(field)) {
            return null;
        }
        try {
            JsonElement el = obj.get(field);
            if (el == null || el.isJsonNull()) {
                return null;
            }
            return el.getAsDouble();
        } catch (Exception e) {
            try {
                return Double.parseDouble(obj.get(field).getAsString());
            } catch (Exception ignored) {
                return null;
            }
        }
    }

    private static long optLong(JsonObject obj, String field, long def) {
        if (obj == null || field == null) {
            return def;
        }
        try {
            JsonElement el = obj.get(field);
            if (el == null) {
                return def;
            }
            if (el.isJsonPrimitive()) {
                if (el.getAsJsonPrimitive().isNumber()) {
                    return el.getAsLong();
                }
                String s = el.getAsString();
                if (s != null && !s.isEmpty()) {
                    return Long.parseLong(s);
                }
            }
        } catch (Exception ignored) {
        }
        return def;
    }

    @Override
    public void close() {
        running.set(false);
        try {
            if (socket != null) {
                socket.close(1000, "bye");
            }
        } catch (Exception ignored) {
        }
        try {
            executor.shutdownNow();
        } catch (Exception ignored) {
        }
        try {
            client.dispatcher().cancelAll();
            client.dispatcher().executorService().shutdown();
            client.connectionPool().evictAll();
        } catch (Exception ignored) {
        }
    }
}
