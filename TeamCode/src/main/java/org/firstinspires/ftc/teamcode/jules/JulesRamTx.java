// org.firstinspires.ftc.teamcode.jules.JulesRamTx
package org.firstinspires.ftc.teamcode.jules;

import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBuffer;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesMetricsHttpAdapter;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesStreamBus;

public class JulesRamTx implements AutoCloseable {
    private final JulesBuffer buffer;
    private final JulesStreamBus streamBus;
    private final TelemetryManager panelsTel;   // Panels bus (optional)
    private final Telemetry dsTelemetry;        // DS telemetry (for flush, optional)
    private final String prefix;

    public JulesRamTx(int capacity, TelemetryManager panelsTelemetry, Telemetry dsTelemetry, String topicPrefix) {
        this(new JulesBuffer(capacity), null, panelsTelemetry, dsTelemetry, topicPrefix);
    }

    public JulesRamTx(JulesBuffer sharedBuffer, JulesStreamBus streamBus,
                      TelemetryManager panelsTelemetry, Telemetry dsTelemetry, String topicPrefix) {
        this.buffer = (sharedBuffer != null) ? sharedBuffer : new JulesBuffer(Math.max(1, 4096));
        this.streamBus = streamBus;
        this.panelsTel = panelsTelemetry;
        this.dsTelemetry = dsTelemetry;
        this.prefix = (topicPrefix != null && !topicPrefix.isEmpty()) ? topicPrefix : "jules";
    }

    public void send(Metrics m) {
        if (m == null) return;
        buffer.push(m);
        if (streamBus != null) {
            streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(m));
        }
        if (panelsTel != null) {
            panelsTel.debug(
                    prefix + "/t: "        + String.format("%.3f", m.t),
                    prefix + "/cmd: "      + String.format("%.2f", m.cmdPower),
                    prefix + "/velIPS: "   + String.format("%.2f", m.velIPS),
                    prefix + "/heading: "  + String.format("%.1f", m.headingDeg),
                    prefix + "/batteryV: " + String.format("%.2f", m.batteryV)
            );
            panelsTel.update(dsTelemetry); // flush to Panels + DS
        }
    }

    public void label(String text) {
        Metrics mark = new Metrics();
        mark.t = System.nanoTime() * 1e-9;
        mark.label = text;
        buffer.push(mark);
        if (streamBus != null) {
            streamBus.publishJsonLine(JulesMetricsHttpAdapter.encodePublic(mark));
        }
        if (panelsTel != null) {
            panelsTel.debug(prefix + "/label: " + text);
            panelsTel.update(dsTelemetry);
        }
    }

    public Metrics[] snapshot(){ return buffer.snapshot(); }
    public int size(){ return buffer.size(); }
    public int capacity(){ return buffer.capacity(); }
    @Override public void close() {}
}