// File: TeamCode/src/main/java/org/firstinspires/ftc/teamcode/jules/bridge/JulesBufferJsonAdapter.java
package org.firstinspires.ftc.teamcode.jules.bridge;

import org.firstinspires.ftc.teamcode.jules.Metrics;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class JulesBufferJsonAdapter implements JulesHttpBridge.Dumper, JulesHttpBridge.Labeler {
    private final JulesBuffer buffer;

    public JulesBufferJsonAdapter(JulesBuffer buffer) {
        this.buffer = buffer;
    }

    /**
     * Dumps all metrics from the buffer as an iterator of JSON strings (JSONL format).
     * @return An iterator where each string is a complete JSON object.
     */
    @Override
    public Iterator<String> dumpAll() {
        List<String> out = new ArrayList<>();
        // We get a snapshot of the buffer to avoid issues with concurrent modification
        for (Metrics m : buffer.snapshot()) {
            if (m != null) {
                // Use the encoder from the metrics adapter for a consistent format
                out.add(JulesMetricsHttpAdapter.encodePublic(m));
            }
        }
        return out.iterator();
    }

    /**
     * Dumps all metrics recorded after a specific timestamp.
     * @param sinceMs The timestamp in milliseconds.
     * @return An iterator of JSON strings for metrics newer than the timestamp.
     */
    @Override
    public Iterator<String> dumpSince(long sinceMs) {
        List<String> out = new ArrayList<>();
        for (Metrics m : buffer.snapshot()) {
            if (m != null) {
                // Convert the metric's time (in seconds) to milliseconds for comparison
                long tMs = Math.round(m.t * 1000.0);
                if (tMs > sinceMs) {
                    out.add(JulesMetricsHttpAdapter.encodePublic(m));
                }
            }
        }
        return out.iterator();
    }

    /**
     * Adds a label to the buffer.
     * @param tMillis The timestamp for the label (provided by the bridge).
     * @param text The text of the label.
     */
    @Override
    public void addLabel(long tMillis, String text) {
        // This now correctly calls the label method we added to JulesBuffer
        buffer.label(text);
    }
}