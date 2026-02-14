package org.firstinspires.ftc.teamcode.jules.link;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
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
        if (heartbeat == null) return;

        JsonObject payload = GsonCompat.deepCopy(heartbeat);  // <-- replace deepCopy()

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
        } catch (Exception ignored) { }
    }


    @Override
    public void close() {
        socket.close();
    }
}

