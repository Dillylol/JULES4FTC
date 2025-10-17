package org.firstinspires.ftc.teamcode.jules.bridge;

import android.content.Context;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.JulesBuilder;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;

import java.io.IOException;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

/**
 * Singleton holder for the persistent JULES bridge components.
 *
 * <p>The manager owns the shared metric buffer, stream bus, HTTP bridge and
 * background heartbeats. OpModes can grab a transmitter/receiver without
 * worrying about when the bridge started; the {@link JulesBridgeSwitch}
 * OpMode is responsible for toggling the connection on and off.</p>
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

        Status(boolean running,
               String ip,
               int port,
               String token,
               String maskedToken,
               long uptimeMs,
               int retryCount,
               String advertiseLine,
               String lastError) {
            this.running = running;
            this.ip = ip;
            this.port = port;
            this.token = token;
            this.maskedToken = maskedToken;
            this.uptimeMs = uptimeMs;
            this.retryCount = retryCount;
            this.advertiseLine = advertiseLine;
            this.lastError = lastError;
        }
    }

    private static final JulesBridgeManager INSTANCE = new JulesBridgeManager();

    private static final int PORT = 58080;
    private static final int DEFAULT_BUFFER_CAPACITY = 8192;
    private static final long HEARTBEAT_PERIOD_MS = 5_000L;

    private final Object lock = new Object();

    private Context appContext;
    private JulesBuffer buffer;
    private JulesStreamBus streamBus;
    private JulesBufferJsonAdapter adapter;
    private JulesHttpBridge httpBridge;
    private ScheduledExecutorService heartbeatExecutor;

    private boolean running;
    private long startTimestampMs;
    private int failureCount;
    private String lastIp;
    private String lastToken;
    private String lastAdvertise;
    private String lastError;

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
        synchronized (lock) {
            this.appContext = context.getApplicationContext();
            if (lastToken == null) {
                lastToken = JulesTokenStore.getOrCreate(this.appContext);
            }
            if (lastIp == null) {
                lastIp = defaultIp();
            }
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
        synchronized (lock) {
            if (running) {
                return true;
            }

            String token = (tokenOverride != null && !tokenOverride.isEmpty())
                    ? tokenOverride
                    : ensureToken(null);
            String ip = (ipOverride != null && !ipOverride.isEmpty())
                    ? ipOverride
                    : defaultIp();

            buffer = new JulesBuffer(DEFAULT_BUFFER_CAPACITY);
            streamBus = new JulesStreamBus();
            adapter = new JulesBufferJsonAdapter(buffer);

            try {
                httpBridge = new JulesHttpBridge(PORT, adapter, adapter, token, streamBus);
                running = true;
                startTimestampMs = System.currentTimeMillis();
                lastToken = token;
                lastIp = ip;
                lastAdvertise = httpBridge.advertiseLine();
                lastError = null;
                startHeartbeatLocked();
                streamBus.publishJsonLine(String.format(Locale.US,
                        "{\"event\":\"bridge_started\",\"ip\":\"%s\",\"port\":%d}", ip, PORT));
                return true;
            } catch (IOException e) {
                failureCount++;
                lastError = e.getMessage();
                stopLocked();
                return false;
            }
        }
    }

    /** Stop the bridge and release resources. */
    public void stop() {
        synchronized (lock) {
            stopLocked();
        }
    }

    private void stopLocked() {
        running = false;
        stopHeartbeatLocked();
        if (httpBridge != null) {
            try {
                httpBridge.close();
            } catch (Exception ignored) {
            }
        }
        if (streamBus != null) {
            try {
                streamBus.close();
            } catch (Exception ignored) {
            }
        }
        httpBridge = null;
        streamBus = null;
        adapter = null;
        buffer = null;
        lastAdvertise = offlineAdvert();
    }

    private void startHeartbeatLocked() {
        stopHeartbeatLocked();
        heartbeatExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "JULES-Heartbeat");
            t.setDaemon(true);
            return t;
        });
        heartbeatExecutor.scheduleAtFixedRate(() -> {
            JulesStreamBus bus;
            long uptime;
            synchronized (lock) {
                if (!running || streamBus == null) {
                    return;
                }
                bus = streamBus;
                uptime = System.currentTimeMillis() - startTimestampMs;
            }
            bus.publishJsonLine(String.format(Locale.US,
                    "{\"heartbeat\":true,\"uptime_ms\":%d}", uptime));
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
                    lastError
            );
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

    public int getPort() {
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
        return String.format(Locale.US, "JULES bridge stopped – IP %s  token=%s", ip, masked);
    }

    private static String randomToken() {
        java.util.UUID uuid = java.util.UUID.randomUUID();
        return Long.toString(uuid.getMostSignificantBits() & Long.MAX_VALUE, 36)
                + Long.toString(uuid.getLeastSignificantBits() & Long.MAX_VALUE, 36);
    }
}
