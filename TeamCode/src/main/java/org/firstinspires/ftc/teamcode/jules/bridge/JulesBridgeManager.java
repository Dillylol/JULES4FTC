package org.firstinspires.ftc.teamcode.jules.bridge;

import android.content.Context;
import android.content.SharedPreferences;

import com.bylazar.telemetry.TelemetryManager;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.JulesBuilder;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.link.JulesUdpBeacon;
import org.firstinspires.ftc.teamcode.jules.opmode.JulesBridgeSwitch;
import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

/**
 * Singleton holder for the persistent JULES bridge components.
 *
 * <p>
 * The manager owns the shared metric buffer, stream bus, HTTP bridge and
 * background heartbeats. OpModes can grab a transmitter/receiver without
 * worrying about when the bridge started; the {@link JulesBridgeSwitch}
 * OpMode is responsible for toggling the connection on and off.
 * </p>
 */
public final class JulesBridgeManager {

    public static final class Status {
        public final boolean running;
        public final String ip;
        public final int port;
        public final String token;
        public final String maskedToken;
        public final long uptimeMs;
        public final int retryCount;
        public final String advertiseLine;
        public final String lastError;
        public final String wsUrl;

        Status(boolean running,
                String ip,
                int port,
                String token,
                String maskedToken,
                long uptimeMs,
                int retryCount,
                String advertiseLine,
                String lastError,
                String wsUrl) {
            this.running = running;
            this.ip = ip;
            this.port = port;
            this.token = token;
            this.maskedToken = maskedToken;
            this.uptimeMs = uptimeMs;
            this.retryCount = retryCount;
            this.advertiseLine = advertiseLine;
            this.lastError = lastError;
            this.wsUrl = wsUrl;
        }
    }

    private static final JulesBridgeManager INSTANCE = new JulesBridgeManager();

    private static final int PORT = 58080;
    private static final int DEFAULT_BUFFER_CAPACITY = 8192;
    private static final long HEARTBEAT_PERIOD_MS = 5_000L;
    private static final String PREFS_NAME = "jules_prefs";
    private static final String PREF_AUTO_ENABLED = "jules_auto_enabled";

    private final Object lock = new Object();

    private Context appContext;
    private JulesBuffer buffer;
    private JulesStreamBus streamBus;
    private JulesBufferJsonAdapter adapter;
    private JulesHttpBridge httpBridge;
    private ScheduledExecutorService heartbeatExecutor;
    private JulesUdpBeacon udpBeacon;

    private boolean running;
    private boolean starting;
    private long startTimestampMs;
    private int failureCount;
    private String lastIp;
    private String lastToken;
    private String lastAdvertise;
    private String lastError;
    private String lastWsUrl;

    // Dynamic Hardware Scanner
    private org.firstinspires.ftc.teamcode.jules.core.JulesHardwareScanner scanner;
    private VoltageSensor batterySensor;

    public void setBatterySensor(VoltageSensor sensor) {
        synchronized (lock) {
            this.batterySensor = sensor;
        }
    }

    public void setHardwareScanner(org.firstinspires.ftc.teamcode.jules.core.JulesHardwareScanner scanner) {
        synchronized (lock) {
            this.scanner = scanner;
        }
    }

    public org.firstinspires.ftc.teamcode.jules.core.JulesHardwareScanner getHardwareScanner() {
        synchronized (lock) {
            return scanner;
        }
    }

    public void publishManifest() {
        JulesStreamBus bus;
        org.firstinspires.ftc.teamcode.jules.core.JulesHardwareScanner s;
        synchronized (lock) {
            if (!running || streamBus == null) {
                return;
            }
            bus = streamBus;
            s = scanner;
        }

        if (s != null) {
            JsonObject manifest = s.getManifest();
            manifest.addProperty("type", "manifest");
            bus.publishJsonLine(manifest.toString());
        }
    }

    private JulesBridgeManager() {
    }

    public static JulesBridgeManager getInstance() {
        return INSTANCE;
    }

    /** Store the application context for token persistence and future starts. */
    public void prepare(Context context) {
        if (context == null) {
            return;
        }

        Context applicationContext = context.getApplicationContext();
        boolean autoStart;
        String ip;
        String token;

        synchronized (lock) {
            this.appContext = applicationContext;
            if (lastToken == null || lastToken.isEmpty()) {
                lastToken = JulesTokenStore.getOrCreate(this.appContext);
            }
            if (lastIp == null || lastIp.isEmpty()) {
                lastIp = defaultIp();
            }
            if (lastWsUrl == null || lastWsUrl.isEmpty()) {
                lastWsUrl = JulesHttpBridge.websocketUrlFor(lastIp, PORT);
            }
            autoStart = isAutoEnabledLocked();
            ip = lastIp;
            token = lastToken;
        }

        if (autoStart) {
            start(ip, token);
        }
    }

    /**
     * Resolve (or create) the bridge token using either the stored
     * application context or the provided fallback context.
     */
    public String ensureToken(Context fallbackContext) {
        synchronized (lock) {
            if (lastToken != null && !lastToken.isEmpty()) {
                return lastToken;
            }
            Context ctx = appContext != null ? appContext
                    : (fallbackContext != null ? fallbackContext.getApplicationContext() : null);
            if (ctx != null) {
                lastToken = JulesTokenStore.getOrCreate(ctx);
            } else {
                lastToken = randomToken();
            }
            return lastToken;
        }
    }

    /** Start the HTTP bridge if it is currently stopped. */
    public boolean start(String ipOverride, String tokenOverride) {
        Callable<Boolean> task;
        synchronized (lock) {
            if (running || starting) {
                return true;
            }

            starting = true;

            final String ip = (ipOverride != null && !ipOverride.isEmpty())
                    ? ipOverride
                    : defaultIp();
            // Auth disabled: always use empty token so all connections are accepted
            final String token = "";

            lastIp = ip;
            lastToken = token;

            task = () -> doStart(ip, token);
        }

        ExecutorService executor = Executors.newSingleThreadExecutor(nonDaemonFactory("JULES-Start"));
        try {
            Future<Boolean> future = executor.submit(task);
            return future.get();
        } catch (Exception e) {
            String message = e.getCause() != null ? e.getCause().getMessage() : e.getMessage();
            synchronized (lock) {
                starting = false;
                failureCount++;
                lastError = message;
                lastAdvertise = offlineAdvert();
            }
            return false;
        } finally {
            executor.shutdown();
        }
    }

    /** Stop the bridge and release resources. */
    public void stop() {
        Callable<Boolean> task;
        synchronized (lock) {
            if (!running && !starting && httpBridge == null && streamBus == null) {
                lastAdvertise = offlineAdvert();
                return;
            }
            task = () -> {
                doStop();
                return true;
            };
        }

        ExecutorService executor = Executors.newSingleThreadExecutor(nonDaemonFactory("JULES-Stop"));
        try {
            Future<Boolean> future = executor.submit(task);
            future.get();
        } catch (Exception e) {
            String message = e.getCause() != null ? e.getCause().getMessage() : e.getMessage();
            synchronized (lock) {
                lastError = message;
            }
        } finally {
            executor.shutdown();
        }
    }

    private void startHeartbeatLocked() {
        stopHeartbeatLocked();
        heartbeatExecutor = Executors.newSingleThreadScheduledExecutor(nonDaemonFactory("JULES-Heartbeat"));
        heartbeatExecutor.scheduleAtFixedRate(() -> {
            JulesStreamBus bus;
            String ip;
            String token;
            long uptime;
            JulesUdpBeacon beacon;
            synchronized (lock) {
                if (!running || streamBus == null) {
                    return;
                }
                bus = streamBus;
                ip = lastIp != null ? lastIp : defaultIp();
                token = lastToken;
                uptime = System.currentTimeMillis() - startTimestampMs;
                uptime = System.currentTimeMillis() - startTimestampMs;
                beacon = udpBeacon;
            }

            double vbatt = 0.0;
            synchronized (lock) {
                if (batterySensor != null) {
                    try {
                        vbatt = batterySensor.getVoltage();
                    } catch (Exception ignored) {
                    }
                }
            }

            long now = System.currentTimeMillis();
            JsonObject heartbeat = new JsonObject();
            heartbeat.addProperty("type", "heartbeat");
            heartbeat.addProperty("ts_ms", now);
            heartbeat.addProperty("vbatt", vbatt);
            heartbeat.addProperty("uptime_ms", uptime);
            heartbeat.addProperty("ip", ip);
            heartbeat.addProperty("port", PORT);
            if (token != null) {
                heartbeat.addProperty("token", token);
            }
            String wsUrl = JulesHttpBridge.websocketUrlFor(ip, PORT);
            heartbeat.addProperty("ws_url", wsUrl);
            bus.publishJsonLine(heartbeat.toString());
            if (beacon != null) {
                beacon.sendHeartbeat(heartbeat, wsUrl);
            }
        }, HEARTBEAT_PERIOD_MS, HEARTBEAT_PERIOD_MS, TimeUnit.MILLISECONDS);
    }

    private void stopHeartbeatLocked() {
        if (heartbeatExecutor != null) {
            heartbeatExecutor.shutdownNow();
            heartbeatExecutor = null;
        }
    }

    /** Provide a transmitter bound to the shared buffer/bus if available. */
    public JulesRamTx getTransmitter(TelemetryManager panelsTelemetry,
            Telemetry dsTelemetry,
            String topicPrefix) {
        synchronized (lock) {
            if (buffer != null && streamBus != null) {
                return new JulesRamTx(buffer, streamBus, panelsTelemetry, dsTelemetry, topicPrefix);
            }
        }
        return new JulesRamTx(DEFAULT_BUFFER_CAPACITY, panelsTelemetry, dsTelemetry, topicPrefix);
    }

    /** Convenience wrapper to build a {@link JulesBuilder} directly. */
    public JulesBuilder newBuilder(TelemetryManager panelsTelemetry,
            Telemetry dsTelemetry,
            String topicPrefix) {
        return new JulesBuilder(getTransmitter(panelsTelemetry, dsTelemetry, topicPrefix));
    }

    /** Return the live stream bus (may be {@code null} when offline). */
    public JulesStreamBus getStreamBus() {
        synchronized (lock) {
            return streamBus;
        }
    }

    /** Return an immutable snapshot of the bridge status for telemetry. */
    public Status getStatusSnapshot() {
        synchronized (lock) {
            long uptime = running ? (System.currentTimeMillis() - startTimestampMs) : 0L;
            return new Status(
                    running,
                    lastIp != null ? lastIp : defaultIp(),
                    PORT,
                    lastToken,
                    maskToken(lastToken),
                    uptime,
                    failureCount,
                    getAdvertiseLineInternal(),
                    lastError,
                    lastWsUrl != null ? lastWsUrl
                            : JulesHttpBridge.websocketUrlFor(lastIp != null ? lastIp : defaultIp(), PORT));
        }
    }

    /** Whether the HTTP bridge is up. */
    public boolean isRunning() {
        synchronized (lock) {
            return running && httpBridge != null;
        }
    }

    public String getAdvertiseLine() {
        synchronized (lock) {
            return getAdvertiseLineInternal();
        }
    }

    private String getAdvertiseLineInternal() {
        if (running && httpBridge != null) {
            lastAdvertise = httpBridge.advertiseLine();
            String ip = lastIp != null ? lastIp : defaultIp();
            lastWsUrl = httpBridge.websocketUrl(ip);
            return lastAdvertise;
        }
        if (lastAdvertise == null) {
            lastAdvertise = offlineAdvert();
        }
        return lastAdvertise;
    }

    public String getToken() {
        synchronized (lock) {
            return lastToken;
        }
    }

    public String getWsUrl() {
        synchronized (lock) {
            if (lastWsUrl == null) {
                String ip = lastIp != null ? lastIp : defaultIp();
                lastWsUrl = JulesHttpBridge.websocketUrlFor(ip, PORT);
            }
            return lastWsUrl;
        }
    }

    public int getPort() {
        return PORT;
    }

    public int port() {
        return PORT;
    }

    public String defaultIp() {
        List<String> ips = JulesHttpBridge.getLocalIPv4();
        return ips.isEmpty() ? "0.0.0.0" : ips.get(0);
    }

    /** Mask a token for telemetry (first/last 4 chars). */
    public static String maskToken(String token) {
        if (token == null || token.isEmpty()) {
            return "n/a";
        }
        if (token.length() <= 4) {
            return token;
        }
        int visible = Math.min(4, token.length() / 2);
        String head = token.substring(0, visible);
        String tail = token.substring(token.length() - visible);
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < token.length() - (2 * visible); i++) {
            sb.append('*');
        }
        return head + sb + tail;
    }

    private String offlineAdvert() {
        String ip = lastIp != null ? lastIp : defaultIp();
        String masked = maskToken(lastToken);
        if (masked == null || masked.isEmpty()) {
            masked = "n/a";
        }
        lastWsUrl = JulesHttpBridge.websocketUrlFor(ip, PORT);
        return String.format(Locale.US,
                "JULES bridge stopped – %s  token=%s",
                lastWsUrl,
                masked);
    }

    private static String randomToken() {
        java.util.UUID uuid = java.util.UUID.randomUUID();
        return Long.toString(uuid.getMostSignificantBits() & Long.MAX_VALUE, 36)
                + Long.toString(uuid.getLeastSignificantBits() & Long.MAX_VALUE, 36);
    }

    private boolean doStart(String ip, String token) {
        JulesBuffer newBuffer = new JulesBuffer(DEFAULT_BUFFER_CAPACITY);
        JulesStreamBus newStreamBus = new JulesStreamBus();
        JulesBufferJsonAdapter newAdapter = new JulesBufferJsonAdapter(newBuffer);
        JulesHttpBridge newBridge;
        JulesUdpBeacon newUdpBeacon;

        try {
            newUdpBeacon = new JulesUdpBeacon();
        } catch (IOException e) {
            newUdpBeacon = null;
        }

        try {
            newBridge = new JulesHttpBridge(PORT, newAdapter, newAdapter, "", newStreamBus); // empty token = no auth
        } catch (IOException e) {
            safeClose(newStreamBus);
            safeClose(newUdpBeacon);
            synchronized (lock) {
                starting = false;
                running = false;
                failureCount++;
                lastError = e.getMessage();
                lastAdvertise = offlineAdvert();
            }
            return false;
        }

        synchronized (lock) {
            buffer = newBuffer;
            streamBus = newStreamBus;
            adapter = newAdapter;
            httpBridge = newBridge;
            udpBeacon = newUdpBeacon;
            running = true;
            starting = false;
            startTimestampMs = System.currentTimeMillis();
            lastToken = token;
            lastIp = ip;
            lastAdvertise = httpBridge.advertiseLine();
            lastWsUrl = httpBridge.websocketUrl(ip);
            lastError = null;
            stopHeartbeatLocked();
            startHeartbeatLocked();
        }

        JsonObject started = new JsonObject();
        started.addProperty("type", "event");
        started.addProperty("event", "bridge_started");
        started.addProperty("ip", ip);
        started.addProperty("port", PORT);
        started.addProperty("ws_url", JulesHttpBridge.websocketUrlFor(ip, PORT));
        started.addProperty("ts_ms", System.currentTimeMillis());
        newStreamBus.publishJsonLine(started.toString());
        return true;
    }

    private void doStop() {
        JulesHttpBridge bridgeToClose;
        JulesStreamBus busToClose;
        JulesUdpBeacon beaconToClose;
        synchronized (lock) {
            running = false;
            starting = false;
            stopHeartbeatLocked();
            bridgeToClose = httpBridge;
            busToClose = streamBus;
            beaconToClose = udpBeacon;
            httpBridge = null;
            streamBus = null;
            adapter = null;
            buffer = null;
            udpBeacon = null;
            lastAdvertise = offlineAdvert();
        }

        safeClose(bridgeToClose);
        safeClose(busToClose);
        safeClose(beaconToClose);
    }

    private void safeClose(AutoCloseable closeable) {
        if (closeable == null) {
            return;
        }
        try {
            closeable.close();
        } catch (Exception ignored) {
        }
    }

    private ThreadFactory nonDaemonFactory(final String name) {
        return r -> {
            Thread t = new Thread(r, name);
            t.setDaemon(false);
            return t;
        };
    }

    private boolean isAutoEnabledLocked() {
        Context ctx = appContext;
        if (ctx == null) {
            return false;
        }
        SharedPreferences prefs = ctx.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        return prefs.getBoolean(PREF_AUTO_ENABLED, false);
    }

    public void setAutoEnabled(boolean enabled) {
        Context ctx;
        synchronized (lock) {
            ctx = appContext;
        }
        if (ctx == null) {
            return;
        }
        SharedPreferences prefs = ctx.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        prefs.edit().putBoolean(PREF_AUTO_ENABLED, enabled).apply();
    }
}
