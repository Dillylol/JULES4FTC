package org.firstinspires.ftc.teamcode.jules.link;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import androidx.annotation.Nullable;

import com.google.gson.JsonObject;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesTelemetry;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import okhttp3.WebSocket;

/**
 * Coordinates telemetry aggregation and link transports for an OpMode lifecycle while keeping
 * the link warm across fast OpMode switches.
 */
public final class JulesLinkManager {

    private static final String PREF_WS_URL = "JULES_WS_URL";
    private static final String DEFAULT_WS_URL = "ws://192.168.43.1:8765/stream";
    private static final long STOP_GRACE_MS = 7_000L;

    private static final Object HOST_LOCK = new Object();
    private static final Object SNAPSHOT_LOCK = new Object();
    private static final ScheduledExecutorService PERMANENCE_EXECUTOR = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread thread = new Thread(r, "JulesLinkPermanence");
        thread.setDaemon(true);
        return thread;
    });

    private static final JulesDataOrganizer ORGANIZER = JulesDataOrganizer.getInstance();

    private static final CopyOnWriteArrayList<Runnable> CONNECTED_CALLBACKS = new CopyOnWriteArrayList<>();

    private static JulesScheduler sharedScheduler;
    private static JulesWsClient sharedWsClient;
    private static JulesUdpBeacon sharedUdpBeacon;
    private static JulesHttpServer sharedHttpServer;
    private static ScheduledFuture<?> pendingStopFuture;
    private static JsonObject lastSnapshotData;
    private static String configuredWsUrl = DEFAULT_WS_URL;
    private static String activeWsUrl = DEFAULT_WS_URL;
    private static boolean transportsRunning;
    private static int attachedOpModes;

    private static final JulesWsClient.Listener WS_EVENT_LISTENER = new JulesWsClient.Listener() {
        @Override
        public void onSocketOpen() {
            handleSocketOpen();
        }

        @Override
        public void onSocketClosed() {
            handleSocketClosed();
        }
    };

    private OpMode opMode;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Telemetry originalTelemetry;
    private JulesTelemetry telemetryProxy;
    private final List<Runnable> ownedConnectedCallbacks = new ArrayList<>();
/**
 * Entry point tying together discovery, scheduling, and transport layers.
 */
public class JulesLinkManager {

    private static final String PREF_WS_URL = "JULES_WS_URL";
    private static final String DEFAULT_WS_URL = "ws://192.168.49.1:8765/stream";

    private final JulesDataOrganizer organizer = JulesDataOrganizer.getInstance();

    private JulesTelemetry telemetryProxy;
    private Telemetry telemetry;
    private JulesWsClient wsClient;
    private JulesUdpBeacon udpBeacon;
    private JulesHttpServer httpServer;
    private JulesScheduler scheduler;
    private String wsUrl;
    private OpMode opMode;

    public void init(OpMode opMode,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamepad1,
                     Gamepad gamepad2) {
        this.opMode = opMode;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.originalTelemetry = telemetry;

        ORGANIZER.bind(hardwareMap, gamepad1, gamepad2, opMode);
        ORGANIZER.setOpModeState("INIT");

        telemetryProxy = new JulesTelemetry(telemetry, ORGANIZER);
        Telemetry proxied = telemetryProxy.getFtcTelemetry();
        if (this.opMode != null && proxied != null) {
            this.opMode.telemetry = proxied;
        }

        String resolvedUrl = resolveWsUrl(opMode);
        synchronized (HOST_LOCK) {
            configuredWsUrl = resolvedUrl;
            ensureTransportsInitializedLocked();
            if (sharedUdpBeacon != null) {
                sharedUdpBeacon.setWebSocketUrl(configuredWsUrl);
            }
            if (!configuredWsUrl.equals(activeWsUrl) && sharedWsClient != null) {
                sharedWsClient.disconnect();
                activeWsUrl = configuredWsUrl;
            }
        }
    }

    public void start() {
        ORGANIZER.setOpModeState("RUNNING");
        synchronized (HOST_LOCK) {
            ensureTransportsInitializedLocked();
            if (sharedHttpServer != null) {
                sharedHttpServer.start();
            }
            if (sharedScheduler == null) {
                sharedScheduler = new JulesScheduler();
                sharedScheduler.scheduleAtFixedRate(JulesLinkManager::sendHeartbeat, 0, 200);
                sharedScheduler.scheduleAtFixedRate(JulesLinkManager::sendSnapshotInternal, 0, 1000);
                sharedScheduler.scheduleAtFixedRate(JulesLinkManager::sendDiff, 100, 100);
            }
            if (sharedUdpBeacon != null && !transportsRunning) {
                sharedUdpBeacon.open();
            }
            transportsRunning = true;
            if (sharedWsClient != null) {
                if (!configuredWsUrl.equals(activeWsUrl)) {
                    sharedWsClient.disconnect();
                    activeWsUrl = configuredWsUrl;
                }
                sharedWsClient.connect(configuredWsUrl);
            }
        }
    }

    public void loop() {
        ORGANIZER.updateGamepads(gamepad1, gamepad2);
    }

    public void stop() {
        performStop();
        restoreTelemetry();
    }

    public void attachOpMode() {
        synchronized (HOST_LOCK) {
            attachedOpModes++;
            cancelPendingStopLocked();
        }
    }

    public void detachOpMode() {
        restoreTelemetry();
        removeOwnedCallbacks();
        synchronized (HOST_LOCK) {
            if (attachedOpModes > 0) {
                attachedOpModes--;
            }
            if (attachedOpModes == 0) {
                scheduleStopLocked();
            }
        }
    }

    public void onConnected(Runnable callback) {
        if (callback == null) {
            return;
        }
        synchronized (HOST_LOCK) {
            CONNECTED_CALLBACKS.add(callback);
            ownedConnectedCallbacks.add(callback);
        }
    }

    public void sendSnapshotNow() {
        sendSnapshotInternal();
    }

    public void sendNdjson(String jsonLine) {
        if (jsonLine == null) {
            return;
        }
        String trimmed = jsonLine.trim();
        if (trimmed.isEmpty()) {
            return;
        }
        JulesWsClient client;
        synchronized (HOST_LOCK) {
            client = sharedWsClient;
        }
        if (client == null || !client.isConnected()) {
            return;
        }
        try {
            Field socketField = JulesWsClient.class.getDeclaredField("socket");
            socketField.setAccessible(true);
            Object socketObj = socketField.get(client);
            if (socketObj instanceof WebSocket) {
                WebSocket socket = (WebSocket) socketObj;
                String payload = trimmed.endsWith("\n") ? trimmed : trimmed + "\n";
                socket.send(payload);
            }
        } catch (Exception ignored) {
        }
    }

    private static void ensureTransportsInitializedLocked() {
        if (sharedWsClient == null) {
            activeWsUrl = configuredWsUrl;
            sharedWsClient = new JulesWsClient(WS_EVENT_LISTENER);
        }
        if (sharedUdpBeacon == null) {
            sharedUdpBeacon = new JulesUdpBeacon();
            sharedUdpBeacon.setWebSocketUrl(configuredWsUrl);
        }
        if (sharedHttpServer == null) {
            sharedHttpServer = new JulesHttpServer(ORGANIZER);
        }
    }

    private static void sendHeartbeat() {
        JsonObject heartbeat = ORGANIZER.buildHeartbeat();
        JulesWsClient client;
        JulesUdpBeacon beacon;
        synchronized (HOST_LOCK) {
            client = sharedWsClient;
            beacon = sharedUdpBeacon;
        }
        if (client != null) {
            client.send(heartbeat);
        }
        if ((client == null || !client.isConnected()) && beacon != null) {
            beacon.sendHeartbeat(heartbeat);
        }
    }

    private static void sendSnapshotInternal() {
        JsonObject snapshot = ORGANIZER.buildSnapshot();
        JsonObject latest = ORGANIZER.getLastDataSnapshot();
        synchronized (SNAPSHOT_LOCK) {
            lastSnapshotData = latest;
        }
        JulesWsClient client;
        synchronized (HOST_LOCK) {
            client = sharedWsClient;
        }
        if (client != null) {
            client.send(snapshot);
        }
    }

    private static void sendDiff() {
        JsonObject baseline;
        synchronized (SNAPSHOT_LOCK) {
            baseline = lastSnapshotData;
        }
        if (baseline == null) {
            return;
        }
        JsonObject diff = ORGANIZER.buildDiffSince(baseline);
        JsonObject latest = ORGANIZER.getLastDataSnapshot();
        synchronized (SNAPSHOT_LOCK) {
            if (latest != null) {
                lastSnapshotData = latest;
            }
        }
        if (diff != null && diff.has("patch") && diff.getAsJsonObject("patch").size() > 0) {
            JulesWsClient client;
            synchronized (HOST_LOCK) {
                client = sharedWsClient;
            }
            if (client != null) {
                client.send(diff);
            }
        }
    }

    private static void handleSocketOpen() {
        sendSnapshotInternal();
        PERMANENCE_EXECUTOR.execute(() -> {
            for (Runnable callback : CONNECTED_CALLBACKS) {
                try {
                    callback.run();
                } catch (Exception ignored) {
                }
            }
        });
    }

    private static void handleSocketClosed() {
        // No-op; reconnect logic lives inside JulesWsClient.
    }

    private void restoreTelemetry() {
        if (opMode != null && originalTelemetry != null) {
            opMode.telemetry = originalTelemetry;
        }
    }

    private void removeOwnedCallbacks() {
        synchronized (HOST_LOCK) {
            if (!ownedConnectedCallbacks.isEmpty()) {
                CONNECTED_CALLBACKS.removeAll(ownedConnectedCallbacks);
                ownedConnectedCallbacks.clear();
            }
        }
    }

    private static void scheduleStopLocked() {
        cancelPendingStopLocked();
        pendingStopFuture = PERMANENCE_EXECUTOR.schedule(JulesLinkManager::performStop, STOP_GRACE_MS, TimeUnit.MILLISECONDS);
    }

    private static void cancelPendingStopLocked() {
        if (pendingStopFuture != null) {
            pendingStopFuture.cancel(false);
            pendingStopFuture = null;
        }
    }

    private static void performStop() {
        synchronized (HOST_LOCK) {
            cancelPendingStopLocked();
            if (sharedScheduler != null) {
                sharedScheduler.shutdown();
                sharedScheduler = null;
            }
            if (sharedWsClient != null) {
                sharedWsClient.shutdown();
                shutdownOkHttp(sharedWsClient);
                sharedWsClient = null;
            }
            if (sharedUdpBeacon != null) {
                sharedUdpBeacon.close();
                sharedUdpBeacon = null;
            }
            if (sharedHttpServer != null) {
                sharedHttpServer.stop();
                sharedHttpServer = null;
            }
            transportsRunning = false;
            lastSnapshotData = null;
            activeWsUrl = configuredWsUrl;
        }
        ORGANIZER.setOpModeState("STOPPED");
    }

    private static void shutdownOkHttp(JulesWsClient client) {
        try {
            Field field = JulesWsClient.class.getDeclaredField("client");
            field.setAccessible(true);
            Object okHttp = field.get(client);
            if (okHttp instanceof okhttp3.OkHttpClient) {
                okhttp3.OkHttpClient okClient = (okhttp3.OkHttpClient) okHttp;
                okClient.dispatcher().executorService().shutdown();
                okClient.connectionPool().evictAll();
            }
        } catch (Exception ignored) {
        }
    }

    private String resolveWsUrl(@Nullable OpMode opMode) {
        if (opMode == null) {
            return DEFAULT_WS_URL;
        }
        Context context = opMode.hardwareMap != null ? opMode.hardwareMap.appContext : null;
        if (context == null) {
            return DEFAULT_WS_URL;
        }
        SharedPreferences preferences = PreferenceManager.getDefaultSharedPreferences(context);
        return preferences.getString(PREF_WS_URL, DEFAULT_WS_URL);
        organizer.bind(opMode, hardwareMap, gamepad1, gamepad2, telemetry);
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.INIT);
        telemetryProxy = new JulesTelemetry(telemetry, organizer);
        this.telemetry = telemetryProxy.getFtcTelemetry();

        Context context = hardwareMap.appContext;
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
        wsUrl = prefs.getString(PREF_WS_URL, DEFAULT_WS_URL);

        wsClient = new JulesWsClient(wsUrl);
        try {
            udpBeacon = new JulesUdpBeacon();
        } catch (Exception ignored) {
            udpBeacon = null;
        }
        httpServer = new JulesHttpServer(organizer);
        scheduler = new JulesScheduler(organizer, wsClient, udpBeacon, httpServer);
    }

    public void start() {
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.RUNNING);
        wsClient.connect();
        scheduler.start();
    }

    public void loop() {
        // no-op; placeholder for future enhancements
    }

    public void stop() {
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.STOPPED);
        scheduler.stop();
        wsClient.close();
        if (udpBeacon != null) {
            udpBeacon.close();
        }
        if (httpServer != null) {
            httpServer.stopServer();
        }
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public String getWsUrl() {
        return wsUrl;
    }
}

