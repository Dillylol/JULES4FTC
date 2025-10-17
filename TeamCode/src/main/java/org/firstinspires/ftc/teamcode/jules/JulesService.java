package org.firstinspires.ftc.teamcode.jules;

import android.content.Context;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBufferJsonAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesHttpBridge;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesTokenStore;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Central lifecycle manager for the persistent JULES data bridge.
 *
 * The service spins up the HTTP bridge once when the Robot Controller boots, keeps
 * the shared buffer/stream bus alive across OpModes, and exposes helper factory
 * methods so each OpMode can obtain a {@link JulesBuilder} wired into the
 * persistent transport layer.
 */
public final class JulesService {

    private static final int DEFAULT_BUFFER_CAPACITY = 8192;
    private static JulesBuffer buffer;
    private static JulesStreamBus streamBus;
    private static JulesBufferJsonAdapter adapter;
    private static JulesHttpBridge httpBridge;
    private static boolean bridgeOnline = false;

    private JulesService() {}

    /**
     * Initializes the JULES bridge if it has not already been started.
     */
    public static synchronized void init(Context context) {
        if (bridgeOnline) {
            return;
        }

        buffer = new JulesBuffer(DEFAULT_BUFFER_CAPACITY);
        streamBus = new JulesStreamBus();
        adapter = new JulesBufferJsonAdapter(buffer);
        String token = JulesTokenStore.getOrCreate(context);

        try {
            httpBridge = new JulesHttpBridge(58080, adapter, adapter, token, streamBus);
            bridgeOnline = true;
        } catch (IOException e) {
            e.printStackTrace();
            shutdownInternals();
        }
    }

    /**
     * Creates a {@link JulesBuilder} bound to the persistent data bridge.
     * If the bridge failed to start, the builder still returns but will be
     * backed by an isolated RAM buffer.
     */
    public static synchronized JulesBuilder newBuilder(TelemetryManager panelsTelemetry,
                                                       Telemetry dsTelemetry,
                                                       String topicPrefix) {
        JulesRamTx tx = newTransmitter(panelsTelemetry, dsTelemetry, topicPrefix);
        return new JulesBuilder(tx);
    }

    /**
     * Creates a transmitter that feeds either the shared buffer/stream bus or a
     * standalone buffer if the service is offline.
     */
    public static synchronized JulesRamTx newTransmitter(TelemetryManager panelsTelemetry,
                                                          Telemetry dsTelemetry,
                                                          String topicPrefix) {
        if (buffer != null) {
            return new JulesRamTx(buffer, streamBus, panelsTelemetry, dsTelemetry, topicPrefix);
        }
        return new JulesRamTx(DEFAULT_BUFFER_CAPACITY, panelsTelemetry, dsTelemetry, topicPrefix);
    }

    /**
     * Adds the HTTP bridge advertisement line (IP + token) to DS telemetry.
     */
    public static void advertise(Telemetry telemetry) {
        if (telemetry == null) return;
        telemetry.addLine(advertiseLine());
    }

    /**
     * Returns a short human-readable advertisement string for telemetry.
     */
    public static synchronized String advertiseLine() {
        if (httpBridge != null) {
            return httpBridge.advertiseLine();
        }
        return "JULES HTTP bridge offline";
    }

    /**
     * @return {@code true} when the persistent HTTP bridge started successfully.
     */
    public static synchronized boolean isBridgeOnline() {
        return bridgeOnline && httpBridge != null;
    }

    public static synchronized String token() {
        return (httpBridge != null) ? httpBridge.getToken() : null;
    }

    public static synchronized void stop() {
        shutdownInternals();
    }

    private static void shutdownInternals() {
        bridgeOnline = false;
        if (httpBridge != null) {
            httpBridge.close();
        }
        if (streamBus != null) {
            streamBus.close();
        }
        httpBridge = null;
        streamBus = null;
        buffer = null;
        adapter = null;
    }
}
