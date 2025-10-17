package org.firstinspires.ftc.teamcode.jules.bridge;

import static fi.iki.elonen.NanoHTTPD.Response.Status.BAD_REQUEST;
import static fi.iki.elonen.NanoHTTPD.Response.Status.INTERNAL_ERROR;
import static fi.iki.elonen.NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED;
import static fi.iki.elonen.NanoHTTPD.Response.Status.NOT_FOUND;
import static fi.iki.elonen.NanoHTTPD.Response.Status.OK;
import static fi.iki.elonen.NanoHTTPD.Response.Status.UNAUTHORIZED;

import fi.iki.elonen.NanoHTTPD;

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
 * JULES HTTP Bridge (NanoHTTPD)
 *
 * Routes:
 *   GET  /jules/handshake            -> { ok, token, time }     (no token required)
 *   GET  /jules/dump[?since=ms]      -> application/jsonl       (requires token)
 *   POST /jules/label?text=...&t=ms  -> { ok }                  (requires token)
 *   GET  /jules/stream               -> Server-Sent Events      (requires token)
 *
 * Token is accepted as header "X-Jules-Token: <token>" OR query "?token=<token>".
 * Start in OpMode.init(), advertise in loop(), close in stop().
 */
public class JulesHttpBridge extends NanoHTTPD implements AutoCloseable {

    /** Supplies JSONL lines from recorded data. */
    public interface Dumper {
        Iterator<String> dumpAll();                 // all lines
        default Iterator<String> dumpSince(long sinceEpochMs) { return null; } // optional incremental
    }

    /** Records a label marker into the dataset. */
    public interface Labeler { void addLabel(long epochMs, String text); }

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

    /** Create with provided token and optional stream bus (enables /jules/stream). */
    public JulesHttpBridge(int port, Dumper dumper, Labeler labeler, String tokenOverride, JulesStreamBus streamBus) throws IOException {
        super(port);
        this.port  = port;
        this.dumper = dumper;
        this.labeler = labeler;
        this.token = (tokenOverride != null) ? tokenOverride : generateToken();
        this.streamBus = streamBus;
        start(NanoHTTPD.SOCKET_READ_TIMEOUT, false);
    }

    public String getToken()     { return token; }
    public int getBoundPort()    { return port; }

    /** One-line DS telemetry string: http://IP:PORT  token=... */
    public String advertiseLine() {
        List<String> ips = getLocalIPv4();
        String ip = ips.isEmpty() ? "0.0.0.0" : ips.get(0);
        return "JULES → http://" + ip + ":" + port + "  token=" + token;
    }

    /** Local site-local IPv4s for telemetry. */
    public static List<String> getLocalIPv4() {
        List<String> ips = new ArrayList<>();
        try {
            Enumeration<NetworkInterface> nets = NetworkInterface.getNetworkInterfaces();
            for (NetworkInterface netIf : Collections.list(nets)) {
                if (!netIf.isUp() || netIf.isLoopback()) continue;
                for (InetAddress addr : Collections.list(netIf.getInetAddresses())) {
                    if (addr instanceof Inet4Address && addr.isSiteLocalAddress()) {
                        ips.add(addr.getHostAddress());
                    }
                }
            }
        } catch (SocketException ignored) { }
        return ips;
    }

    // ------------------------- HTTP -------------------------

    @Override
    public Response serve(IHTTPSession session) {
        // Parse body so POST form params appear in session.getParms()
        Map<String,String> body = new HashMap<>();
        try { session.parseBody(body); } catch (Exception e) {
            return json(BAD_REQUEST, "{\"ok\":false,\"error\":\"body parse\"}");
        }

        final Method method = session.getMethod();
        final String uri = session.getUri();
        final Map<String,String> params = session.getParms();
        final Map<String,String> headers = session.getHeaders();
        final String postData = body.get("postData");

        final boolean isHandshake = method == Method.GET && "/jules/handshake".equals(uri);
        if (!isHandshake && !authorized(params, headers)) {
            return json(UNAUTHORIZED, "{\"ok\":false,\"error\":\"unauthorized\"}");
        }

        try {
            // Handshake
            if (isHandshake) {
                String json = "{\"ok\":true,\"token\":\"" + token + "\",\"time\":\"" + nowIso() + "\"}";
                return json(OK, json);
            }

            // Dump
            if (method == Method.GET && "/jules/dump".equals(uri)) {
                long since = parseLong(params.get("since"), -1);
                Iterator<String> it = (since > 0) ? dumper.dumpSince(since) : null;
                if (it == null) it = dumper.dumpAll();
                if (it == null) return json(INTERNAL_ERROR, "{\"ok\":false,\"error\":\"no dumper\"}");

                StringBuilder sb = new StringBuilder(4096);
                while (it.hasNext()) {
                    String line = it.next();
                    if (line == null || line.isEmpty()) continue;
                    if (!line.endsWith("\n")) sb.append(line).append('\n'); else sb.append(line);
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
                if (labeler != null) labeler.addLabel(t, text);
                return json(OK, "{\"ok\":true}");
            }

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
                if (streamBus != null) {
                    streamBus.publishJsonLine("{\"command\":\"" + escape(sanitized) + "\"}");
                }
                return json(OK, "{\"ok\":true, \"command_sent\":\"" + escape(sanitized) + "\"}");
            }

            // Live stream (SSE)
            if ("/jules/stream".equals(uri)) {
                if (method != Method.GET) {
                    return json(METHOD_NOT_ALLOWED, "{\"ok\":false,\"error\":\"method\"}");
                }
                if (streamBus == null) {
                    return json(NOT_FOUND, "{\"ok\":false,\"error\":\"stream not enabled\"}");
                }
                try {
                    final java.io.PipedOutputStream pos = new java.io.PipedOutputStream();
                    final java.io.PipedInputStream  pis = new java.io.PipedInputStream(pos, 8192);
                    final JulesStreamBus.Subscription sub = streamBus.subscribe();

                    Thread t = new Thread(() -> {
                        final java.io.OutputStreamWriter w = new java.io.OutputStreamWriter(pos);
                        final long heartbeatMs = 5000; 
                        long lastBeat = System.currentTimeMillis();
                        try {
                            w.write(":\n\n"); w.flush();
                            while (true) {
                                long now = System.currentTimeMillis();
                                if (now - lastBeat >= heartbeatMs) {
                                    w.write(":\n\n"); w.flush();
                                    lastBeat = now;
                                }
                                String line = sub.take();
                                if (line == null) break;
                                w.write("data: "); w.write(line); w.write("\n\n");
                                w.flush();
                            }
                        } catch (Exception ignored) {
                        } finally {
                            try { sub.close(); } catch (Exception ignored) {}
                            try { w.close(); } catch (Exception ignored) {}
                            try { pos.close(); } catch (Exception ignored) {}
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

    private static String firstNonEmpty(String a, String b) {
        if (a != null && !a.trim().isEmpty()) return a;
        if (b != null && !b.trim().isEmpty()) return b;
        return null;
    }

    // ------------------------- Helpers -------------------------

    private boolean authorized(Map<String,String> params, Map<String,String> headers) {
        String q = params.get("token");
        String h = headers.get("x-jules-token");
        return token.equals(q) || token.equals(h);
    }

    private static long parseLong(String s, long def) {
        try { return (s == null) ? def : Long.parseLong(s); } catch (Exception e) { return def; }
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
                +  Long.toString(UUID.randomUUID().getLeastSignificantBits() & Long.MAX_VALUE, 36).substring(0, 4);
    }

    private static Response json(Response.IStatus status, String json) {
        return newFixedLengthResponse(status, "application/json; charset=utf-8", json);
    }

    @Override public void close() { try { stop(); } catch (Exception ignored) {} }

    // -------------------- Convenience factory --------------------

    /** Build a Dumper from a supplier of an iterator of JSONL lines. */
    public static Dumper dumperFrom(Supplier<Iterator<String>> allSupplier) {
        return new Dumper() {
            @Override public Iterator<String> dumpAll() { return allSupplier.get(); }
        };
    }

    private static final Pattern JSON_COMMAND_PATTERN = Pattern.compile("\\\"command\\\"\\s*:\\s*\\\"([^\\\"]+)\\\"");

    private static String extractCommandFromBody(String body) {
        if (body == null) {
            return null;
        }

        String trimmed = body.trim();
        if (trimmed.isEmpty()) {
            return null;
        }

        Matcher matcher = JSON_COMMAND_PATTERN.matcher(trimmed);
        if (matcher.find()) {
            return matcher.group(1);
        }

        String decoded = urlDecode(trimmed);
        if (!decoded.equals(trimmed)) {
            Matcher decodedMatcher = JSON_COMMAND_PATTERN.matcher(decoded);
            if (decodedMatcher.find()) {
                return decodedMatcher.group(1);
            }
        }

        String fromKv = extractFromKeyValue(trimmed);
        if (fromKv != null) {
            return fromKv;
        }

        if (!decoded.equals(trimmed)) {
            fromKv = extractFromKeyValue(decoded);
            if (fromKv != null) {
                return fromKv;
            }
        }

        if (trimmed.startsWith("\"") && trimmed.endsWith("\"") && trimmed.length() > 1) {
            return trimmed.substring(1, trimmed.length() - 1);
        }
        if (!decoded.equals(trimmed) && decoded.startsWith("\"") && decoded.endsWith("\"") && decoded.length() > 1) {
            return decoded.substring(1, decoded.length() - 1);
        }

        return decoded.trim();
    }

    private static String extractFromKeyValue(String candidate) {
        String[] tokens = candidate.split("&");
        for (String token : tokens) {
            int idx = token.indexOf('=');
            if (idx <= 0) {
                continue;
            }
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
