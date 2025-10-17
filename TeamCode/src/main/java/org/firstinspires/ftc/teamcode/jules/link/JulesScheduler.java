package org.firstinspires.ftc.teamcode.jules.link;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

/**
 * Single-threaded scheduler for Jules link tasks.
 */
public final class JulesScheduler {

    private final ScheduledExecutorService executor;
    private final List<ScheduledFuture<?>> futures = new ArrayList<>();

    public JulesScheduler() {
        this.executor = Executors.newSingleThreadScheduledExecutor(new ThreadFactory() {
            @Override
            public Thread newThread(@NonNull Runnable r) {
                Thread thread = new Thread(r, "JulesLinkScheduler");
                thread.setDaemon(true);
                return thread;
            }
        });
    }

    public synchronized ScheduledFuture<?> scheduleAtFixedRate(Runnable task, long initialDelayMs, long periodMs) {
        ScheduledFuture<?> future = executor.scheduleAtFixedRate(task, initialDelayMs, periodMs, TimeUnit.MILLISECONDS);
        futures.add(future);
        return future;
    }

    public synchronized void shutdown() {
        for (ScheduledFuture<?> future : futures) {
            if (future != null) {
                future.cancel(true);
            }
        }
        futures.clear();
        executor.shutdownNow();
import com.google.gson.JsonObject;

import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;

import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * Periodically emits heartbeat, snapshot, and diff frames.
 */
public class JulesScheduler implements JulesWsClient.ConnectionListener, AutoCloseable {

    private final JulesDataOrganizer organizer;
    private final JulesWsClient wsClient;
    private final JulesUdpBeacon udpBeacon;
    private final JulesHttpServer httpServer;
    private final ScheduledExecutorService executor;

    private final AtomicBoolean started = new AtomicBoolean(false);
    private volatile JsonObject lastSnapshotData;

    private ScheduledFuture<?> heartbeatFuture;
    private ScheduledFuture<?> snapshotFuture;
    private ScheduledFuture<?> diffFuture;

    public JulesScheduler(JulesDataOrganizer organizer,
                          JulesWsClient wsClient,
                          JulesUdpBeacon udpBeacon,
                          JulesHttpServer httpServer) {
        this.organizer = organizer;
        this.wsClient = wsClient;
        this.udpBeacon = udpBeacon;
        this.httpServer = httpServer;
        this.executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "JulesScheduler");
            t.setDaemon(true);
            return t;
        });
        this.wsClient.setConnectionListener(this);
    }

    public void start() {
        if (!started.compareAndSet(false, true)) {
            return;
        }
        if (httpServer != null) {
            httpServer.startServer();
        }
        lastSnapshotData = organizer.getLatestSnapshotData();
        heartbeatFuture = executor.scheduleAtFixedRate(this::safeHeartbeat, 0, 200, TimeUnit.MILLISECONDS);
        snapshotFuture = executor.scheduleAtFixedRate(this::safeSnapshot, 0, 1000, TimeUnit.MILLISECONDS);
        diffFuture = executor.scheduleAtFixedRate(this::safeDiff, 100, 100, TimeUnit.MILLISECONDS);
    }

    public void sendImmediateSnapshot() {
        executor.execute(() -> {
            JsonObject snapshot = organizer.buildSnapshot();
            lastSnapshotData = snapshot.getAsJsonObject("data");
            wsClient.send(snapshot);
        });
    }

    private void safeHeartbeat() {
        try {
            JsonObject heartbeat = organizer.buildHeartbeat();
            wsClient.send(heartbeat);
            if (!wsClient.isOpen() && udpBeacon != null) {
                udpBeacon.sendHeartbeat(heartbeat, wsClient.getUrl());
            }
        } catch (Exception ignored) {
            // never crash scheduler
        }
    }

    private void safeSnapshot() {
        try {
            JsonObject snapshot = organizer.buildSnapshot();
            lastSnapshotData = snapshot.getAsJsonObject("data");
            wsClient.send(snapshot);
        } catch (Exception ignored) {
        }
    }

    private void safeDiff() {
        try {
            JsonObject baseline = lastSnapshotData;
            if (baseline == null) {
                return;
            }
            JsonObject diff = organizer.buildDiffSince(baseline);
            if (diff != null) {
                lastSnapshotData = organizer.getLatestSnapshotData();
                wsClient.send(diff);
            }
        } catch (Exception ignored) {
        }
    }

    @Override
    public void onOpen() {
        sendImmediateSnapshot();
    }

    @Override
    public void onClosed() {
        // no-op
    }

    public void stop() {
        if (!started.compareAndSet(true, false)) {
            return;
        }
        if (heartbeatFuture != null) heartbeatFuture.cancel(true);
        if (snapshotFuture != null) snapshotFuture.cancel(true);
        if (diffFuture != null) diffFuture.cancel(true);
        executor.shutdownNow();
        if (httpServer != null) {
            httpServer.stopServer();
        }
    }

    @Override
    public void close() {
        stop();
    }
}

