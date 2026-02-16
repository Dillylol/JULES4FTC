package org.firstinspires.ftc.teamcode.jules.link;

import android.util.Log;

import com.google.gson.Gson;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Map;
import org.firstinspires.ftc.teamcode.jules.bridge.util.GsonCompat; // if you haven’t added it yet

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CopyOnWriteArrayList;
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
 * Handles outbound WebSocket connection to the telemetry aggregator service.
 */
public class JulesWsClient {

    public interface ConnectionListener {
        void onOpen();

        void onClosed();
    }

    public interface CommandListener {
        void onCommand(String type, JsonObject payload);
    }

    private static final String TAG = "JulesWsClient";
    private static final int MAX_QUEUE_SIZE = 256;

    private final OkHttpClient client;
    private final ScheduledExecutorService executor;
    private final Queue<String> outbound = new ConcurrentLinkedQueue<>();
    private final Gson gson = new Gson();
    private final String url;

    private volatile WebSocket socket;
    private final AtomicBoolean open = new AtomicBoolean(false);
    private final AtomicBoolean running = new AtomicBoolean(false);
    private int reconnectAttempts = 0;
    private final CopyOnWriteArrayList<ConnectionListener> listeners = new CopyOnWriteArrayList<>();
    private final CopyOnWriteArrayList<CommandListener> commandListeners = new CopyOnWriteArrayList<>();

    public JulesWsClient(String url) {
        this.url = url;
        this.executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "JulesWsClient");
            t.setDaemon(true);
            return t;
        });
        this.client = new OkHttpClient.Builder()
                .readTimeout(0, TimeUnit.MILLISECONDS)
                .build();
    }

    public void setConnectionListener(ConnectionListener listener) {
        listeners.clear();
        if (listener != null) {
            listeners.add(listener);
        }
    }

    public void addConnectionListener(ConnectionListener listener) {
        if (listener != null) {
            listeners.addIfAbsent(listener);
        }
    }

    public void removeConnectionListener(ConnectionListener listener) {
        if (listener != null) {
            listeners.remove(listener);
        }
    }

    public void addCommandListener(CommandListener listener) {
        if (listener != null) {
            commandListeners.addIfAbsent(listener);
        }
    }

    public void removeCommandListener(CommandListener listener) {
        if (listener != null) {
            commandListeners.remove(listener);
        }
    }

    public void connect() {
        if (running.compareAndSet(false, true)) {
            scheduleConnect(0);
        }
    }

    public void close() {
        running.set(false);
        try {
            if (socket != null) {
                socket.close(1000, "bye");
            }
        } catch (Exception ignored) {
        }
        socket = null;
        open.set(false);
        executor.shutdownNow();
        try {
            client.dispatcher().cancelAll();
            client.dispatcher().executorService().shutdown();
            client.connectionPool().evictAll();
        } catch (Exception ignored) {
        }
    }

    public boolean isOpen() {
        return open.get();
    }

    public String getUrl() {
        return url;
    }

    public void send(JsonObject message) {
        if (message == null) {
            return;
        }
        sendRaw(gson.toJson(message) + "\n");
    }

    public void sendRaw(String payload) {
        if (payload == null) {
            return;
        }
        if (outbound.size() > MAX_QUEUE_SIZE) {
            outbound.poll();
        }
        outbound.offer(payload);
        flushIfOpen();
    }

    private void flushIfOpen() {
        if (!open.get()) {
            return;
        }
        String next;
        while ((next = outbound.poll()) != null) {
            WebSocket ws = socket;
            if (ws != null) {
                ws.send(next);
            }
        }
    }

    private void scheduleConnect(long delayMs) {
        if (executor.isShutdown()) {
            return;
        }
        executor.schedule(this::doConnect, delayMs, TimeUnit.MILLISECONDS);
    }

    private void doConnect() {
        if (!running.get()) {
            return;
        }
        Request request = new Request.Builder().url(url).build();
        socket = client.newWebSocket(request, new WebSocketListener() {
            @Override
            public void onOpen(WebSocket webSocket, Response response) {
                open.set(true);
                reconnectAttempts = 0;
                flushIfOpen();
                for (ConnectionListener l : listeners) {
                    try {
                        l.onOpen();
                    } catch (Exception e) {
                        RobotLog.ee(TAG, e, "ConnectionListener.onOpen failure");
                    }
                }
            }

            @Override
            public void onMessage(WebSocket webSocket, String text) {
                handleInbound(text);
            }

            @Override
            public void onClosed(WebSocket webSocket, int code, String reason) {
                open.set(false);
                for (ConnectionListener l : listeners) {
                    try {
                        l.onClosed();
                    } catch (Exception e) {
                        RobotLog.ee(TAG, e, "ConnectionListener.onClosed failure");
                    }
                }
                scheduleReconnect();
            }

            @Override
            public void onFailure(WebSocket webSocket, Throwable t, Response response) {
                open.set(false);
                RobotLog.ee(TAG, t, "WebSocket failure: %s", t.getMessage());
                for (ConnectionListener l : listeners) {
                    try {
                        l.onClosed();
                    } catch (Exception e) {
                        RobotLog.ee(TAG, e, "ConnectionListener.onClosed failure");
                    }
                }
                scheduleReconnect();
            }
        });
    }

    private void scheduleReconnect() {
        if (!running.get()) {
            return;
        }
        long delay = (long) Math.min(5000, Math.pow(2, reconnectAttempts) * 250);
        reconnectAttempts++;
        scheduleConnect(delay);
    }

    private void handleInbound(String text) {
        if (text == null) {
            return;
        }
        try {
            JsonObject obj = GsonCompat.parse(text).getAsJsonObject();
            String type = obj.has("type") ? obj.get("type").getAsString() : "";
            if ("ping".equals(type)) {
                JsonObject pong = new JsonObject();
                for (Map.Entry<String, JsonElement> e : obj.entrySet()) {
                    if (e.getValue() != null) {
                        pong.add(e.getKey(), e.getValue());
                    }
                }
                pong.addProperty("type", "pong");
                pong.addProperty("ts_ms", System.currentTimeMillis());
                send(pong);
            } else if ("cmd".equals(type)) {
                RobotLog.ii(TAG, "Received cmd frame: %s", text);
                sendAck(obj);
            } else {
                // Dispatch other commands (e.g. exec_pyscript, stop_robot)
                for (CommandListener l : commandListeners) {
                    try {
                        l.onCommand(type, obj);
                    } catch (Exception ex) {
                        RobotLog.ee(TAG, ex, "CommandListener.onCommand failure");
                    }
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "Failed to parse inbound WS message", e);
        }
    }

    private void sendAck(JsonObject obj) {
        JsonObject ack = new JsonObject();
        ack.addProperty("type", "stdout");
        ack.addProperty("ts_ms", System.currentTimeMillis());
        ack.addProperty("message", "ack");
        send(ack);
    }
}
