package org.firstinspires.ftc.teamcode.jules.bridge;

import static fi.iki.elonen.NanoHTTPD.Response.Status.BAD_REQUEST;
import static fi.iki.elonen.NanoHTTPD.Response.Status.INTERNAL_ERROR;
import static fi.iki.elonen.NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED;
import static fi.iki.elonen.NanoHTTPD.Response.Status.NOT_FOUND;
import static fi.iki.elonen.NanoHTTPD.Response.Status.OK;
import static fi.iki.elonen.NanoHTTPD.Response.Status.UNAUTHORIZED;

import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonSyntaxException;

import fi.iki.elonen.NanoHTTPD; // base HTTP server (provided by RobotCore)
import fi.iki.elonen.NanoHTTPD.IHTTPSession;
import fi.iki.elonen.NanoHTTPD.Response;
import fi.iki.elonen.NanoHTTPD.Method;
import fi.iki.elonen.NanoWSD; // WebSocket server (from nanohttpd-websocket)
import fi.iki.elonen.NanoWSD.WebSocket;
import fi.iki.elonen.NanoWSD.WebSocketFrame;
import fi.iki.elonen.NanoWSD.WebSocketFrame.CloseCode;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import java.io.IOException;
import java.net.Inet4Address;
import java.net.InetAddress;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.net.URLDecoder;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.Enumeration;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.HashMap;
import java.util.TimeZone;
import java.util.UUID;
import java.util.function.Supplier;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * JULES HTTP Bridge (NanoHTTPD + NanoWSD)
 *
 * Routes:
 * GET /jules/handshake -> { ok, token, time }
 * GET /jules/dump[?since=ms] -> application/jsonl
 * POST /jules/label?text=...&t=ms -> { ok }
 * GET /jules/stream -> Server-Sent Events (SSE)
 * WS /jules/stream -> WebSocket (text frames)
 *
 * Token is accepted as header "X-Jules-Token: <token>" OR query
 * "?token=<token>".
 * Start in OpMode.init(), advertise in loop(), close in stop().
 */
public class JulesHttpBridge extends NanoWSD implements AutoCloseable {

    /** Supplies JSONL lines from recorded data. */
    public interface Dumper {
        Iterator<String> dumpAll(); // all lines

        default Iterator<String> dumpSince(long sinceEpochMs) {
            return null;
        } // optional incremental
    }

    /** Records a label marker into the dataset. */
    public interface Labeler {
        void addLabel(long epochMs, String text);
    }

    private static final String WS_PATH = "/jules/stream";

    private final Dumper dumper;
    private final Labeler labeler;
    private final String token;
    private final int port;
    private final JulesStreamBus streamBus; // may be null

    // ---------------- Constructors (single, canonical set) ----------------

    /** Create with random token; no live stream. */
    public JulesHttpBridge(int port, Dumper dumper, Labeler labeler) throws IOException {
        this(port, dumper, labeler, null, null);
    }

    /** Create with provided token; no live stream. */
    public JulesHttpBridge(int port, Dumper dumper, Labeler labeler, String tokenOverride) throws IOException {
        this(port, dumper, labeler, tokenOverride, null);
    }

    /**
     * Create with provided token and optional stream bus (enables /jules/stream).
     */
    public JulesHttpBridge(int port, Dumper dumper, Labeler labeler, String tokenOverride, JulesStreamBus streamBus)
            throws IOException {
        super(port);
        this.port = port;
        this.dumper = dumper;
        this.labeler = labeler;
        this.token = (tokenOverride != null) ? tokenOverride : generateToken();
        this.streamBus = streamBus;
        // Use 0 timeout so RobotCore's event loop isn't blocked by lingering sockets
        start(0, false);
    }

    public String getToken() {
        return token;
    }

    public int getBoundPort() {
        return port;
    }

    /** One-line DS telemetry string advertising the WS endpoint. */
    public String advertiseLine() {
        List<String> ips = getLocalIPv4();
        String ip = ips.isEmpty() ? "0.0.0.0" : ips.get(0);
        return "JULES → " + websocketUrlFor(ip, port) + "  token=" + token;
    }

    private String firstLocalIp() {
        List<String> ips = getLocalIPv4();
        return ips.isEmpty() ? "0.0.0.0" : ips.get(0);
    }

    public String websocketUrl(String host) {
        String safeHost = (host == null || host.isEmpty()) ? "0.0.0.0" : host;
        return websocketUrlFor(safeHost, port);
    }

    public static String websocketUrlFor(String host, int port) {
        String safeHost = (host == null || host.isEmpty()) ? "0.0.0.0" : host;
        return "ws://" + safeHost + ":" + port + WS_PATH;
    }

    /** Local site-local IPv4s for telemetry. */
    public static List<String> getLocalIPv4() {
        List<String> ips = new ArrayList<>();
        try {
            Enumeration<NetworkInterface> nets = NetworkInterface.getNetworkInterfaces();
            for (NetworkInterface netIf : Collections.list(nets)) {
                if (!netIf.isUp() || netIf.isLoopback())
                    continue;
                for (InetAddress addr : Collections.list(netIf.getInetAddresses())) {
                    if (addr instanceof Inet4Address && addr.isSiteLocalAddress()) {
                        ips.add(addr.getHostAddress());
                    }
                }
            }
        } catch (SocketException ignored) {
        }
        return ips;
    }

    // ------------------------- HTTP -------------------------

    @Override
    protected Response serveHttp(final IHTTPSession session) {
        // Parse body so POST form params appear in session.getParms()
        Map<String, String> body = new HashMap<>();
        try {
            session.parseBody(body);
        } catch (Exception e) {
            return json(BAD_REQUEST, "{\"ok\":false,\"error\":\"body parse\"}");
        }

        final Method method = session.getMethod();
        final String uri = session.getUri();
        final Map<String, String> params = session.getParms();
        final Map<String, String> headers = session.getHeaders();
        final String postData = body.get("postData");

        final boolean isHandshake = method == Method.GET && "/jules/handshake".equals(uri);
        if (!isHandshake && !authorized(params, headers)) {
            return json(UNAUTHORIZED, "{\"ok\":false,\"error\":\"unauthorized\"}");
        }

        try {
            // Handshake
            if (isHandshake) {
                JsonObject payload = new JsonObject();
                payload.addProperty("ok", true);
                payload.addProperty("token", token);
                payload.addProperty("time", nowIso());
                String ip = firstLocalIp();
                payload.addProperty("ws_url", websocketUrlFor(ip, port));
                payload.addProperty("http_url", "http://" + ip + ":" + port);
                return json(OK, payload.toString());
            }

            // Dump
            if (method == Method.GET && "/jules/dump".equals(uri)) {
                long since = parseLong(params.get("since"), -1);
                Iterator<String> it = (since > 0) ? dumper.dumpSince(since) : null;
                if (it == null)
                    it = dumper.dumpAll();
                if (it == null)
                    return json(INTERNAL_ERROR, "{\"ok\":false,\"error\":\"no dumper\"}");

                StringBuilder sb = new StringBuilder(4096);
                while (it.hasNext()) {
                    String line = it.next();
                    if (line == null || line.isEmpty())
                        continue;
                    if (!line.endsWith("\n"))
                        sb.append(line).append('\n');
                    else
                        sb.append(line);
                }
                return newFixedLengthResponse(OK, "application/jsonl; charset=utf-8", sb.toString());
            }

            // Label
            if ("/jules/label".equals(uri)) {
                if (method != Method.GET && method != Method.POST) {
                    return json(METHOD_NOT_ALLOWED, "{\"ok\":false,\"error\":\"method\"}");
                }
                String text = firstNonEmpty(params.get("text"), params.get("label"));
                if (text == null || text.trim().isEmpty()) {
                    return json(BAD_REQUEST, "{\"ok\":false,\"error\":\"missing text\"}");
                }
                long t = parseLong(params.get("t"), System.currentTimeMillis());
                if (labeler != null)
                    labeler.addLabel(t, text);
                publishLabelEvent(text.trim(), t, "http");
                JsonObject response = new JsonObject();
                response.addProperty("ok", true);
                response.addProperty("type", "label");
                response.addProperty("text", text.trim());
                response.addProperty("ts_ms", t);
                return json(OK, response.toString());
            }

            // Command (HTTP)
            if ("/jules/command".equals(uri)) {
                if (method != Method.POST) {
                    return json(METHOD_NOT_ALLOWED, "{\"ok\":false,\"error\":\"method not allowed, use POST\"}");
                }
                String commandText = firstNonEmpty(params.get("command"), extractCommandFromBody(postData));
                if (commandText == null || commandText.trim().isEmpty()) {
                    return json(BAD_REQUEST, "{\"ok\":false,\"error\":\"missing 'command' parameter\"}");
                }
                String sanitized = commandText.trim();
                JulesCommand.setCommand(sanitized);
                long ts = System.currentTimeMillis();
                publishCommandEvent(sanitized, ts, "http");
                JsonObject response = new JsonObject();
                response.addProperty("ok", true);
                response.addProperty("type", "cmd");
                response.addProperty("text", sanitized);
                response.addProperty("ts_ms", ts);
                return json(OK, response.toString());
            }

            // Live stream (SSE over HTTP). WebSocket upgrades are handled in
            // openWebSocket().
            if ("/jules/stream".equals(uri)) {
                if (method != Method.GET) {
                    return json(METHOD_NOT_ALLOWED, "{\"ok\":false,\"error\":\"method\"}");
                }
                if (streamBus == null) {
                    return json(NOT_FOUND, "{\"ok\":false,\"error\":\"stream not enabled\"}");
                }
                try {
                    final java.io.PipedOutputStream pos = new java.io.PipedOutputStream();
                    final java.io.PipedInputStream pis = new java.io.PipedInputStream(pos, 8192);
                    final JulesStreamBus.Subscription sub = streamBus.subscribe();

                    Thread t = new Thread(() -> {
                        final java.io.OutputStreamWriter w = new java.io.OutputStreamWriter(pos);
                        final long heartbeatMs = 5000;
                        long lastBeat = System.currentTimeMillis();
                        try {
                            w.write(":\n\n");
                            w.flush();
                            while (true) {
                                long now = System.currentTimeMillis();
                                if (now - lastBeat >= heartbeatMs) {
                                    w.write(":\n\n");
                                    w.flush();
                                    lastBeat = now;
                                }
                                String line = sub.take();
                                if (line == null)
                                    break;
                                w.write("data: ");
                                w.write(line);
                                w.write("\n\n");
                                w.flush();
                            }
                        } catch (Exception ignored) {
                        } finally {
                            try {
                                sub.close();
                            } catch (Exception ignored) {
                            }
                            try {
                                w.close();
                            } catch (Exception ignored) {
                            }
                            try {
                                pos.close();
                            } catch (Exception ignored) {
                            }
                        }
                    }, "JulesSSEWriter");
                    t.setDaemon(true);
                    t.start();

                    Response r = newChunkedResponse(OK, "text/event-stream; charset=utf-8", pis);
                    r.addHeader("Cache-Control", "no-cache");
                    r.addHeader("Connection", "keep-alive");
                    r.addHeader("Access-Control-Allow-Origin", "*");
                    return r;
                } catch (Exception e) {
                    return json(INTERNAL_ERROR, "{\"ok\":false,\"error\":\"stream setup\"}");
                }
            }

            // 404
            return json(NOT_FOUND, "{\"ok\":false,\"error\":\"not found\"}");
        } catch (Exception e) {
            String msg = e.getClass().getSimpleName() + ": " + (e.getMessage() == null ? "" : e.getMessage());
            return json(INTERNAL_ERROR, "{\"ok\":false,\"error\":\"" + escape(msg) + "\"}");
        }
    }

    @Override
    protected WebSocket openWebSocket(IHTTPSession handshake) {
        return new BridgeWebSocket(handshake);
    }

    private final class BridgeWebSocket extends WebSocket {
        private JulesStreamBus.Subscription subscription;
        private Thread pumpThread;
        private ScheduledExecutorService pinger; // per-connection keepalive

        private BridgeWebSocket(IHTTPSession handshakeRequest) {
            super(handshakeRequest);
        }

        @Override
        protected void onOpen() {
            IHTTPSession request = getHandshakeRequest();
            if (!authorized(request.getParms(), request.getHeaders())) {
                try {
                    close(CloseCode.PolicyViolation, "unauthorized", false);
                } catch (IOException ignored) {
                }
                return;
            }
            if (streamBus == null) {
                sendErrorFrame("stream_unavailable", "stream not enabled", null);
                try {
                    // Use any CloseCode your NanoWSD supports; fallback shown below
                    close(CloseCode.InternalServerError, "stream not enabled", false);
                    // close(CloseCode.AbnormalClosure, "stream not enabled", false);
                } catch (IOException ignored) {
                }
                return;
            }

            subscription = streamBus.subscribe();

            pumpThread = new Thread(this::pump, "JulesWsPump" + System.identityHashCode(this));
            pumpThread.setDaemon(true);
            pumpThread.start();

            sendAnnounceFrame();

            // --- keep the socket warm so we don't hit idle read timeouts ---
            pinger = Executors.newSingleThreadScheduledExecutor(r -> {
                Thread t = new Thread(r, "JulesWsPing");
                t.setDaemon(true);
                return t;
            });
            pinger.scheduleAtFixedRate(() -> {
                try {
                    ping(new byte[0]);
                } catch (Exception ignored) {
                }
            }, 10, 10, TimeUnit.SECONDS);
        }

        private void pump() {
            try {
                while (true) {
                    String line = (subscription != null) ? subscription.take() : null;
                    if (line == null)
                        break;
                    safeSend(line);
                }
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }

        @Override
        protected void onMessage(WebSocketFrame message) {
            if (message == null || message.getOpCode() != WebSocketFrame.OpCode.Text) {
                return;
            }
            handleIncoming(message.getTextPayload());
        }

        private void handleIncoming(String payload) {
            if (payload == null)
                return;
            String trimmed = payload.trim();
            if (trimmed.isEmpty())
                return;

            JsonObject obj;
            try {
                JsonElement parsed = GsonCompat.parse(trimmed);
                if (!parsed.isJsonObject()) {
                    sendErrorFrame("bad_payload", "expected object", null);
                    return;
                }
                obj = parsed.getAsJsonObject();
            } catch (JsonSyntaxException ex) {
                sendErrorFrame("bad_json", ex.getMessage(), null);
                return;
            }

            String type = optString(obj, "type");
            if (type == null) {
                sendErrorFrame("missing_type", "frame missing 'type'", obj);
                return;
            }
            switch (type.toLowerCase(Locale.US)) {
                case "cmd":
                case "command":
                    handleCommandFrame(obj);
                    break;
                case "label":
                    handleLabelFrame(obj);
                    break;
                case "ping":
                    handlePingFrame(obj);
                    break;
                default:
                    sendErrorFrame("unsupported_type", type, obj);
                    break;
            }
        }

        private void handleCommandFrame(JsonObject obj) {
            String text = optString(obj, "text");
            if (text == null)
                text = optString(obj, "command");
            if (text == null && obj.has("payload")) {
                JsonElement payload = obj.get("payload");
                if (payload.isJsonObject() || payload.isJsonArray()) {
                    text = payload.toString();
                } else if (payload.isJsonPrimitive()) {
                    text = payload.getAsString();
                }
            }
            // Fallback: name/args spread at top level (e.g. { type:"cmd", name:"drive",
            // args:{} })
            if (text == null && obj.has("name")) {
                JsonObject cmdObj = new JsonObject();
                cmdObj.add("name", obj.get("name"));
                if (obj.has("args"))
                    cmdObj.add("args", obj.get("args"));
                text = cmdObj.toString();
            }

            if (text == null || text.trim().isEmpty()) {
                sendErrorFrame("missing_command", "command text/payload missing", obj);
                return;
            }
            String sanitized = text.trim();

            // Special Command: QUERY_MANIFEST
            if ("QUERY_MANIFEST".equals(sanitized)) {
                org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager mgr = org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager
                        .getInstance();
                if (mgr != null) {
                    org.firstinspires.ftc.teamcode.jules.core.JulesHardwareScanner scanner = mgr.getHardwareScanner();
                    if (scanner != null) {
                        JsonObject manifest = scanner.getManifest();
                        manifest.addProperty("type", "manifest");
                        manifest.addProperty("ts_ms", System.currentTimeMillis());
                        safeSend(manifest.toString());
                        return;
                    }
                }
                sendErrorFrame("manifest_unavailable", "No hardware scanner registered", obj);
                return;
            }

            JulesCommand.setCommand(sanitized);
            long ts = System.currentTimeMillis();
            publishCommandEvent(sanitized, ts, "ws");
            sendAckFrame("cmd", sanitized, ts, obj);
        }

        private void handleLabelFrame(JsonObject obj) {
            String text = optString(obj, "text");
            if (text == null || text.trim().isEmpty()) {
                sendErrorFrame("missing_text", "label text missing", obj);
                return;
            }
            long ts = obj.has("ts_ms") && obj.get("ts_ms").isJsonPrimitive()
                    ? obj.get("ts_ms").getAsLong()
                    : System.currentTimeMillis();
            publishLabelEvent(text.trim(), ts, "ws");
            if (labeler != null) {
                labeler.addLabel(ts, text.trim());
            }
            sendAckFrame("label", text.trim(), ts, obj);
        }

        private void handlePingFrame(JsonObject obj) {
            long now = System.currentTimeMillis();
            JsonObject pong = new JsonObject();
            pong.addProperty("type", "pong");
            pong.addProperty("ts_ms", now);
            if (obj.has("id"))
                pong.add("id", obj.get("id"));
            if (obj.has("t0"))
                pong.add("t0", obj.get("t0"));
            pong.addProperty("t1", now);
            safeSend(pong.toString());
        }

        private void sendAnnounceFrame() {
            JsonObject announce = new JsonObject();
            announce.addProperty("type", "announce");
            long now = System.currentTimeMillis();
            announce.addProperty("ts_ms", now);
            String ip = firstLocalIp();
            announce.addProperty("ws_url", websocketUrlFor(ip, port));
            announce.addProperty("http_url", "http://" + ip + ":" + port);
            announce.addProperty("token", token);
            safeSend(announce.toString());
        }

        private void sendAckFrame(String action, String text, long ts, JsonObject request) {
            JsonObject ack = new JsonObject();
            ack.addProperty("type", "ack");
            ack.addProperty("action", action);
            ack.addProperty("ts_ms", System.currentTimeMillis());
            ack.addProperty("command_ts_ms", ts);
            if (text != null)
                ack.addProperty("text", text);
            if (request != null) {
                if (request.has("id"))
                    ack.add("id", request.get("id"));
                if (request.has("t0"))
                    ack.add("t0", request.get("t0"));
            }
            safeSend(ack.toString());
        }

        private void sendErrorFrame(String code, String message, JsonObject request) {
            JsonObject error = new JsonObject();
            error.addProperty("type", "error");
            error.addProperty("code", code);
            if (message != null)
                error.addProperty("message", message);
            error.addProperty("ts_ms", System.currentTimeMillis());
            if (request != null && request.has("id"))
                error.add("id", request.get("id"));
            safeSend(error.toString());
        }

        private void safeSend(String text) {
            try {
                send(text);
            } catch (IOException ignored) {
            }
        }

        @Override
        protected void onClose(CloseCode code, String reason, boolean initiatedByRemote) {
            if (subscription != null) {
                subscription.close();
                subscription = null;
            }
            if (pumpThread != null) {
                pumpThread.interrupt();
                pumpThread = null;
            }
            if (pinger != null) {
                pinger.shutdownNow();
                pinger = null;
            }
        }

        @Override
        protected void onPong(WebSocketFrame pong) {
            /* ignored */ }

        @Override
        protected void onException(IOException exception) {
            if (pinger != null) {
                pinger.shutdownNow();
                pinger = null;
            }
            sendErrorFrame("io_exception", exception.getMessage(), null);
        }
    }

    private static String firstNonEmpty(String a, String b) {
        if (a != null && !a.trim().isEmpty())
            return a;
        if (b != null && !b.trim().isEmpty())
            return b;
        return null;
    }

    // ------------------------- Helpers -------------------------

    private static String optString(JsonObject obj, String key) {
        if (obj == null || key == null || !obj.has(key))
            return null;
        JsonElement element = obj.get(key);
        if (element == null || element.isJsonNull())
            return null;
        try {
            return element.getAsString();
        } catch (ClassCastException | IllegalStateException ignored) {
            return null;
        }
    }

    private void publishCommandEvent(String command, long timestampMs, String source) {
        if (streamBus == null || command == null || command.isEmpty())
            return;
        JsonObject event = new JsonObject();
        event.addProperty("type", "cmd");
        event.addProperty("ts_ms", timestampMs);
        event.addProperty("text", command);
        if (source != null && !source.isEmpty())
            event.addProperty("source", source);
        streamBus.publishJsonLine(event.toString());
    }

    private void publishLabelEvent(String text, long timestampMs, String source) {
        if (streamBus == null || text == null || text.isEmpty())
            return;
        JsonObject event = new JsonObject();
        event.addProperty("type", "label");
        event.addProperty("ts_ms", timestampMs);
        event.addProperty("text", text);
        if (source != null && !source.isEmpty())
            event.addProperty("source", source);
        streamBus.publishJsonLine(event.toString());
    }

    private boolean authorized(Map<String, String> params, Map<String, String> headers) {
        if (token == null || token.isEmpty())
            return true;
        String q = params.get("token");
        if (token.equals(q))
            return true;
        String headerToken = headers.get("x-jules-token");
        if (token.equals(headerToken))
            return true;
        String auth = headers.get("authorization");
        if (auth != null) {
            String normalized = auth.trim();
            if (normalized.regionMatches(true, 0, "Bearer ", 0, 7)) {
                String bearer = normalized.substring(7).trim();
                if (token.equals(bearer))
                    return true;
            }
        }
        return false;
    }

    private static long parseLong(String s, long def) {
        try {
            return (s == null) ? def : Long.parseLong(s);
        } catch (Exception e) {
            return def;
        }
    }

    private static String nowIso() {
        SimpleDateFormat fmt = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'", Locale.US);
        fmt.setTimeZone(TimeZone.getTimeZone("UTC"));
        return fmt.format(new Date());
    }

    private static String escape(String s) {
        return s.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n");
    }

    private static String generateToken() {
        return Long.toString(UUID.randomUUID().getMostSignificantBits() & Long.MAX_VALUE, 36)
                + Long.toString(UUID.randomUUID().getLeastSignificantBits() & Long.MAX_VALUE, 36).substring(0, 4);
    }

    private static Response json(Response.IStatus status, String json) {
        return newFixedLengthResponse(status, "application/json; charset=utf-8", json);
    }

    @Override
    public void close() {
        try {
            stop();
        } catch (Exception ignored) {
        }
    }

    // -------------------- Convenience factory --------------------

    /** Build a Dumper from a supplier of an iterator of JSONL lines. */
    public static Dumper dumperFrom(Supplier<Iterator<String>> allSupplier) {
        return new Dumper() {
            @Override
            public Iterator<String> dumpAll() {
                return allSupplier.get();
            }
        };
    }

    private static final Pattern JSON_COMMAND_PATTERN = Pattern.compile("\\\"command\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");

    private static String extractCommandFromBody(String body) {
        if (body == null)
            return null;
        String trimmed = body.trim();
        if (trimmed.isEmpty())
            return null;

        Matcher matcher = JSON_COMMAND_PATTERN.matcher(trimmed);
        if (matcher.find())
            return matcher.group(1);

        String decoded = urlDecode(trimmed);
        if (!decoded.equals(trimmed)) {
            Matcher decodedMatcher = JSON_COMMAND_PATTERN.matcher(decoded);
            if (decodedMatcher.find())
                return decodedMatcher.group(1);
        }

        String fromKv = extractFromKeyValue(trimmed);
        if (fromKv != null)
            return fromKv;
        if (!decoded.equals(trimmed)) {
            fromKv = extractFromKeyValue(decoded);
            if (fromKv != null)
                return fromKv;
        }

        if (trimmed.startsWith("\"") && trimmed.endsWith("\"") && trimmed.length() > 1)
            return trimmed.substring(1, trimmed.length() - 1);
        if (!decoded.equals(trimmed) && decoded.startsWith("\"") && decoded.endsWith("\"") && decoded.length() > 1)
            return decoded.substring(1, decoded.length() - 1);

        return decoded.trim();
    }

    private static String extractFromKeyValue(String candidate) {
        String[] tokens = candidate.split("&");
        for (String token : tokens) {
            int idx = token.indexOf('=');
            if (idx <= 0)
                continue;
            String key = token.substring(0, idx).trim();
            if ("command".equalsIgnoreCase(key)) {
                return urlDecode(token.substring(idx + 1).trim());
            }
        }
        return null;
    }

    private static String urlDecode(String value) {
        try {
            return URLDecoder.decode(value, "UTF-8");
        } catch (Exception ignored) {
            return value;
        }
    }
}
