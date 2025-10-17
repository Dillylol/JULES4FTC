package org.firstinspires.ftc.teamcode.jules.link;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import okhttp3.OkHttpClient;
import okhttp3.Request;
import okhttp3.Response;
import okhttp3.WebSocket;
import okhttp3.WebSocketListener;

/**
 * Resilient outbound WebSocket client that streams NDJSON frames.
 */
public final class JulesWsClient {

    public interface Listener {
        void onSocketOpen();

        void onSocketClosed();
    }

    private static final long INITIAL_BACKOFF_MS = 500L;
    private static final long MAX_BACKOFF_MS = 5000L;

    private final OkHttpClient client = new OkHttpClient();
    private final JsonParser jsonParser = new JsonParser();
    private final AtomicBoolean connected = new AtomicBoolean(false);
    private final ScheduledExecutorService reconnectExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread thread = new Thread(r, "JulesWsReconnect");
        thread.setDaemon(true);
        return thread;
    });

    private final Listener listener;
    private WebSocket socket;
    private String url;
    private long backoffMs = INITIAL_BACKOFF_MS;

    public JulesWsClient(Listener listener) {
        this.listener = listener;
    }

    public synchronized void connect(String url) {
        this.url = url;
        this.backoffMs = INITIAL_BACKOFF_MS;
        attemptConnect();
    }

    public synchronized void disconnect() {
        if (socket != null) {
            socket.close(1000, "shutdown");
            socket = null;
        }
        connected.set(false);
    }

    public void shutdown() {
        disconnect();
        reconnectExecutor.shutdownNow();
    }

    public boolean isConnected() {
        return connected.get();
    }

    public void send(JsonObject payload) {
        if (payload == null) {
            return;
        }
        WebSocket active;
        synchronized (this) {
            active = socket;
        }
        if (active != null && connected.get()) {
            active.send(payload.toString() + "\n");
        }
    }

    private void attemptConnect() {
        if (url == null || url.isEmpty()) {
            return;
        }
        if (socket != null) {
            return;
        }
        Request request = new Request.Builder().url(url).build();
        socket = client.newWebSocket(request, new InternalWebSocketListener());
    }

    private void handleInbound(String text) {
        if (text == null || text.isEmpty()) {
            return;
        }
        String[] lines = text.split("\n");
        for (String line : lines) {
            line = line.trim();
            if (line.isEmpty()) {
                continue;
            }
            try {
                JsonElement element = jsonParser.parse(line);
                if (!element.isJsonObject()) {
                    continue;
                }
                JsonObject obj = element.getAsJsonObject();
                String type = obj.has("type") ? obj.get("type").getAsString() : "";
                if ("ping".equals(type)) {
                    JsonObject pong = new JsonObject();
                    pong.addProperty("type", "pong");
                    if (obj.has("id")) {
                        pong.add("id", obj.get("id"));
                    }
                    if (obj.has("t0")) {
                        pong.add("t0", obj.get("t0"));
                    }
                    send(pong);
                } else if ("cmd".equals(type)) {
                    JsonObject ack = new JsonObject();
                    ack.addProperty("type", "stdout");
                    ack.addProperty("line", "ack");
                    send(ack);
                }
            } catch (Exception ignored) {
            }
        }
    }

    private final class InternalWebSocketListener extends WebSocketListener {

        @Override
        public void onOpen(WebSocket webSocket, Response response) {
            connected.set(true);
            backoffMs = INITIAL_BACKOFF_MS;
            if (listener != null) {
                listener.onSocketOpen();
            }
        }

        @Override
        public void onMessage(WebSocket webSocket, String text) {
            handleInbound(text);
        }

        @Override
        public void onClosed(WebSocket webSocket, int code, String reason) {
            handleClosed();
        }

        @Override
        public void onClosing(WebSocket webSocket, int code, String reason) {
            webSocket.close(code, reason);
            handleClosed();
        }

        @Override
        public void onFailure(WebSocket webSocket, Throwable t, Response response) {
            handleClosed();
        }

        private void handleClosed() {
            connected.set(false);
            if (listener != null) {
                listener.onSocketClosed();
            }
            synchronized (JulesWsClient.this) {
                socket = null;
            }
            scheduleReconnect();
        }
    }

    private void scheduleReconnect() {
        if (url == null || url.isEmpty()) {
            return;
        }
        reconnectExecutor.schedule(this::attemptConnect, backoffMs, TimeUnit.MILLISECONDS);
        backoffMs = Math.min(backoffMs * 2, MAX_BACKOFF_MS);
    }
}

