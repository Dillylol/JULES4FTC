package org.firstinspires.ftc.teamcode.jules.link;

import android.util.Log;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicLong;
import okhttp3.*;

public class JulesWsLink {
    private final OkHttpClient http = new OkHttpClient();
    private final String wsUrl;                  // e.g., "ws://192.168.43.1:8765/stream"
    private final Telemetry tel;                 // optional: to surface link status on DS
    private final ScheduledExecutorService exec = Executors.newSingleThreadScheduledExecutor();
    private WebSocket socket;
    private volatile boolean open = false;
    private final AtomicLong seq = new AtomicLong(0);

    public JulesWsLink(String wsUrl, Telemetry tel) {
        this.wsUrl = wsUrl;
        this.tel = tel;
    }

    public void connect() {
        Request req = new Request.Builder().url(wsUrl).build();
        socket = http.newWebSocket(req, new WebSocketListener() {
            @Override public void onOpen(WebSocket ws, Response resp) {
                open = true;
                if (tel != null) tel.addLine("JULES link: OPEN");
            }
            @Override public void onMessage(WebSocket ws, String text) {
                // Handle pings from client
                if (text.contains("\"type\":\"ping\"")) {
                    ws.send(text.replace("\"type\":\"ping\"", "\"type\":\"pong\""));
                }
            }
            @Override public void onClosed(WebSocket ws, int code, String reason) { open = false; }
            @Override public void onFailure(WebSocket ws, Throwable t, Response resp) { open = false; }
        });

        // Periodic heartbeat @ 5 Hz
        exec.scheduleAtFixedRate(() -> {
            if (!open) return;
            // A tiny payload keeps UI alive even when robot is idle
        }, 0, 200, TimeUnit.MILLISECONDS);
    }

    public void sendHeartbeat(OpMode opMode, VoltageSensor batt) {
        if (!open || socket == null) return;
        long now = System.currentTimeMillis();
        double volts = (batt != null) ? batt.getVoltage() : Double.NaN;
        String active = (opMode != null) ? opMode.getClass().getSimpleName() : "NoOpMode";

        String json = String.format(
                "{\"type\":\"heartbeat\",\"ts_ms\":%d,\"seq\":%d," +
                        "\"battery_v\":%.3f,\"active_opmode\":\"%s\"}\n",
                now, seq.incrementAndGet(), volts, active
        );
        socket.send(json);
    }

    public void close() {
        try { if (socket != null) socket.close(1000, "bye"); } catch (Exception ignored) {}
        exec.shutdownNow();
    }
}
