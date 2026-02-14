package org.firstinspires.ftc.teamcode.jules;

import com.google.gson.Gson;
import java.util.HashMap;
import java.util.Map;

/**
 * A helper class to build and send dynamic key-value data through the Jules system.
 * This provides a similar interface to FTCLib's Panels telemetry.
 */
public class JulesBuilder {

    private final JulesRamTx tx;
    private final Map<String, Object> data = new HashMap<>();
    private static final Gson gson = new Gson(); // Re-use Gson instance for efficiency

    public JulesBuilder(JulesRamTx transmitter) {
        this.tx = transmitter;
    }

    /**
     * Adds a key-value data point to the current data packet.
     * @param key   The name of the data point.
     * @param value The value of the data point (can be any type).
     * @return The JulesBuilder instance for method chaining.
     */
    public JulesBuilder addData(String key, Object value) {
        data.put(key, value);
        return this;
    }

    /**
     * Finalizes the data packet, converts it to JSON, and sends it.
     * The internal data map is cleared after sending.
     * @param runtime The current OpMode runtime, used for the timestamp.
     */
    public void send(double runtime) {
        if (tx == null) return;

        // Create a Metrics object to send through the existing pipeline
        Metrics metrics = new Metrics();
        metrics.t = runtime; // Timestamp is essential
        metrics.jsonData = gson.toJson(data); // Convert our map to a JSON string

        // Send it
        tx.send(metrics);

        // Clear for the next loop iteration
        data.clear();
    }
}