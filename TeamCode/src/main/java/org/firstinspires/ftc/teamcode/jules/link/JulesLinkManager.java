package org.firstinspires.ftc.teamcode.jules.link;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesTelemetry;

import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Entry point tying together discovery, scheduling, and transport layers.
 */
public class JulesLinkManager implements JulesWsClient.ConnectionListener, JulesWsClient.CommandListener {

    private static final String PREF_WS_URL = "JULES_WS_URL";
    private static final String DEFAULT_WS_URL = "ws://192.168.49.1:8765/jules";

    private static final Object GLOBAL_LOCK = new Object();
    private static final long STOP_DELAY_MS = 7000L;
    private static final ScheduledExecutorService STOP_EXECUTOR = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r, "JulesLinkManagerStopper");
        t.setDaemon(true);
        return t;
    });

    private static int attachedCount = 0;
    private static ScheduledFuture<?> pendingStopTask;
    private static SharedResources sharedResources;

    private final JulesDataOrganizer organizer = JulesDataOrganizer.getInstance();

    private final List<Runnable> onConnectedCallbacks = new CopyOnWriteArrayList<>();
    private final AtomicBoolean listenerRegistered = new AtomicBoolean(false);

    private JulesTelemetry telemetryProxy;
    private Telemetry telemetry;
    private SharedResources shared;
    private String wsUrl;
    private OpMode opMode;

    private static final class SharedResources {
        private final JulesWsClient wsClient;
        private final JulesUdpBeacon udpBeacon;
        private final JulesHttpServer httpServer;
        private final JulesScheduler scheduler;

        private SharedResources(String wsUrl, JulesDataOrganizer organizer) {
            this.wsClient = new JulesWsClient(wsUrl);
            JulesUdpBeacon beacon;
            try {
                beacon = new JulesUdpBeacon();
            } catch (Exception ignored) {
                beacon = null;
            }
            this.udpBeacon = beacon;
            this.httpServer = new JulesHttpServer(organizer);
            this.scheduler = new JulesScheduler(organizer, wsClient, udpBeacon, httpServer);
        }
    }

    public void init(OpMode opMode,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            Gamepad gamepad1,
            Gamepad gamepad2) {
        this.opMode = opMode;
        organizer.bind(opMode, hardwareMap, gamepad1, gamepad2, telemetry);
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.INIT);
        telemetryProxy = new JulesTelemetry(telemetry, organizer);
        this.telemetry = telemetryProxy.getFtcTelemetry();

        Context context = hardwareMap.appContext;
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
        wsUrl = prefs.getString(PREF_WS_URL, DEFAULT_WS_URL);

        shared = ensureSharedResources(wsUrl, organizer);
    }

    public void start() {
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.RUNNING);
        registerConnectionListener();
        if (shared != null) {
            shared.wsClient.connect();
            shared.scheduler.start();
        }
    }

    public void loop() {
        // no-op; placeholder for future enhancements
    }

    public void stop() {
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.STOPPED);
        unregisterConnectionListener();
        detachOpMode();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public String getWsUrl() {
        if (shared != null) {
            return shared.wsClient.getUrl();
        }
        return wsUrl;
    }

    public void attachOpMode() {
        synchronized (GLOBAL_LOCK) {
            attachedCount++;
            if (pendingStopTask != null) {
                pendingStopTask.cancel(false);
                pendingStopTask = null;
            }
        }
    }

    public void detachOpMode() {
        boolean noOpModesRemaining = false;
        synchronized (GLOBAL_LOCK) {
            if (attachedCount > 0) {
                attachedCount--;
            }
            if (attachedCount == 0) {
                noOpModesRemaining = true;
                if (sharedResources != null && pendingStopTask == null) {
                    pendingStopTask = STOP_EXECUTOR.schedule(JulesLinkManager::performStop, STOP_DELAY_MS,
                            TimeUnit.MILLISECONDS);
                }
            }
        }
        if (noOpModesRemaining) {
            onConnectedCallbacks.clear();
        }
    }

    public void onConnected(Runnable callback) {
        if (callback == null) {
            return;
        }
        onConnectedCallbacks.add(callback);
        if (shared != null && shared.wsClient.isOpen()) {
            safeRun(callback);
        }
    }

    public void sendNdjson(String payload) {
        if (payload == null || payload.isEmpty()) {
            return;
        }
        SharedResources current = shared;
        if (current == null) {
            return;
        }
        String framed = payload.endsWith("\n") ? payload : payload + "\n";
        current.wsClient.sendRaw(framed);
    }

    public void sendSnapshotNow() {
        SharedResources current = shared;
        if (current != null) {
            current.scheduler.sendImmediateSnapshot();
        }
    }

    @Override
    public void onOpen() {
        for (Runnable callback : onConnectedCallbacks) {
            safeRun(callback);
        }
    }

    @Override
    public void onClosed() {
        // no-op
    }

    @Override
    public void onCommand(String type, com.google.gson.JsonObject payload) {
        if ("exec_pyscript".equals(type)) {
            com.qualcomm.robotcore.util.RobotLog.ii("JulesLinkManager",
                    "Received start command. Simulating RUNNING state.");
            organizer.setOpModeState(JulesDataOrganizer.OpModeState.RUNNING);
        } else if ("stop_robot".equals(type)) {
            com.qualcomm.robotcore.util.RobotLog.ii("JulesLinkManager",
                    "Received stop command. Simulating STOPPED state.");
            organizer.setOpModeState(JulesDataOrganizer.OpModeState.STOPPED);
        } else if ("ping".equals(type)) {
            if (payload.has("ts")) {
                long ts = payload.get("ts").getAsLong();
                com.google.gson.JsonObject pong = new com.google.gson.JsonObject();
                pong.addProperty("type", "pong");
                pong.addProperty("ts", ts);
                sendNdjson(pong.toString());
            }
        }
    }

    private void registerConnectionListener() {
        SharedResources current = shared;
        if (current != null && listenerRegistered.compareAndSet(false, true)) {
            current.wsClient.addConnectionListener(this);
            current.wsClient.addCommandListener(this);
        }
    }

    private void unregisterConnectionListener() {
        SharedResources current = shared;
        if (current != null && listenerRegistered.compareAndSet(true, false)) {
            current.wsClient.removeConnectionListener(this);
            current.wsClient.removeCommandListener(this);
        }
        onConnectedCallbacks.clear();
    }

    public void addCommandListener(JulesWsClient.CommandListener listener) {
        SharedResources current = shared;
        if (current != null) {
            current.wsClient.addCommandListener(listener);
        }
    }

    private static SharedResources ensureSharedResources(String wsUrl, JulesDataOrganizer organizer) {
        synchronized (GLOBAL_LOCK) {
            if (sharedResources == null || !sharedResources.wsClient.getUrl().equals(wsUrl)) {
                if (sharedResources != null) {
                    if (pendingStopTask != null) {
                        pendingStopTask.cancel(false);
                        pendingStopTask = null;
                    }
                    forceStop(sharedResources);
                }
                sharedResources = new SharedResources(wsUrl, organizer);
            }
            return sharedResources;
        }
    }

    private static void performStop() {
        SharedResources toStop;
        synchronized (GLOBAL_LOCK) {
            if (attachedCount != 0) {
                return;
            }
            toStop = sharedResources;
            sharedResources = null;
            pendingStopTask = null;
        }
        if (toStop != null) {
            forceStop(toStop);
        }
    }

    private static void forceStop(SharedResources resources) {
        if (resources == null) {
            return;
        }
        resources.scheduler.stop();
        resources.wsClient.close();
        if (resources.udpBeacon != null) {
            resources.udpBeacon.close();
        }
        if (resources.httpServer != null) {
            resources.httpServer.stopServer();
        }
    }

    private void safeRun(Runnable runnable) {
        try {
            runnable.run();
        } catch (Exception ignored) {
        }
    }
}
