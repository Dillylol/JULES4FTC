package org.firstinspires.ftc.teamcode.jules.bridge;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;

/**
 * Minimal manager that starts/stops the HTTP bridge and advertises to DS telemetry.
 * Panels publishing was removed to avoid API mismatches with third-party telemetry libs.
 */
public final class JulesBridgeManager implements AutoCloseable {
    private JulesHttpBridge http;
    private String token;

    public JulesBridgeManager() {
    }

    /** Start the HTTP bridge on the given port (58080 by default) using your dumper/labeler. */
    public void startWithBuffer(JulesHttpBridge.Dumper dumper, JulesHttpBridge.Labeler labeler) {
        startWithBuffer(58080, dumper, labeler);
    }

    public void startWithBuffer(int port, JulesHttpBridge.Dumper dumper, JulesHttpBridge.Labeler labeler) {
        try {
            http = new JulesHttpBridge(port, dumper, labeler);
            token = http.getToken();
        } catch (IOException e) {
            // Surface in DS telemetry in your OpMode if desired
            token = null;
        }
    }

    /** Add the advertise line to Driver Station telemetry (call from loop()). */
    public void advertiseToDs(Telemetry ds) {
        if (http != null && ds != null) {
            ds.addLine(http.advertiseLine());
        }
    }
    private void publishToPanels(Object panels, String key, String value) {
        if (panels == null) return;
        try {
            // try put(String,String)
            panels.getClass().getMethod("put", String.class, String.class).invoke(panels, key, value);
            return;
        } catch (Exception ignored) {}
        try {
            // try set(String,String)
            panels.getClass().getMethod("set", String.class, String.class).invoke(panels, key, value);
        } catch (Exception ignored) {}
    }

    public String token() { return token; }

    @Override
    public void close() {
        if (http != null) http.close();
    }
}
