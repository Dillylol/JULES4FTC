package org.firstinspires.ftc.teamcode.jules.link;

import androidx.annotation.NonNull;

import com.google.gson.Gson;
import com.google.gson.JsonObject;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;

/**
 * UDP heartbeat broadcaster used as a transport fallback.
 */
public final class JulesUdpBeacon {

    private static final int PORT = 27182;

    private DatagramSocket socket;
    private String wsUrl = "";

    public JulesUdpBeacon() {
    }

    public synchronized void setWebSocketUrl(String url) {
        this.wsUrl = url;
    }

    public synchronized void open() {
        if (socket != null && !socket.isClosed()) {
            return;
        }
        try {
            socket = new DatagramSocket();
            socket.setBroadcast(true);
        } catch (SocketException ignored) {
            socket = null;
        }
    }

    public synchronized void close() {
        if (socket != null) {
            socket.close();
            socket = null;
        }
    }

    public void sendHeartbeat(@NonNull JsonObject heartbeat) {
        DatagramSocket activeSocket;
        synchronized (this) {
            activeSocket = this.socket;
        }
        if (activeSocket == null || activeSocket.isClosed()) {
            return;
        }

        JsonObject payload = heartbeat.deepCopy();
        payload.addProperty("ws_url", wsUrl);
        byte[] data = (payload.toString() + "\n").getBytes(StandardCharsets.UTF_8);
        try {
            DatagramPacket packet = new DatagramPacket(data, data.length, InetAddress.getByName("255.255.255.255"), PORT);
            activeSocket.send(packet);
        } catch (IOException ignored) {
        }
    }
import java.nio.charset.StandardCharsets;

/**
 * Broadcasts UDP heartbeats when WebSocket transport is unavailable.
 */
public class JulesUdpBeacon implements AutoCloseable {

    private static final int PORT = 27182;

    private final DatagramSocket socket;
    private final Gson gson = new Gson();

    public JulesUdpBeacon() throws IOException {
        socket = new DatagramSocket();
        socket.setBroadcast(true);
    }

    public void sendHeartbeat(JsonObject heartbeat, String wsUrl) {
        if (heartbeat == null) {
            return;
        }
        JsonObject payload = heartbeat.deepCopy();
        payload.addProperty("ws_url", wsUrl);
        payload.addProperty("transport", "udp");
        byte[] data = gson.toJson(payload).concat("\n").getBytes(StandardCharsets.UTF_8);
        try {
            DatagramPacket packet = new DatagramPacket(
                    data,
                    data.length,
                    InetAddress.getByName("255.255.255.255"),
                    PORT
            );
            socket.send(packet);
        } catch (Exception ignored) {
            // best effort
        }
    }

    @Override
    public void close() {
        socket.close();
    }
}

