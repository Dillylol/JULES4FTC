package org.firstinspires.ftc.teamcode.jules;

import android.content.Context;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

/**
 * Backwards-compatible facade that delegates to {@link JulesBridgeManager}.
 */
public final class JulesService {

    private JulesService() {
    }

    public static void init(Context context) {
        JulesBridgeManager.getInstance().prepare(context);
    }

    public static JulesBuilder newBuilder(TelemetryManager panelsTelemetry,
                                          Telemetry dsTelemetry,
                                          String topicPrefix) {
        return JulesBridgeManager.getInstance().newBuilder(panelsTelemetry, dsTelemetry, topicPrefix);
    }

    public static JulesRamTx newTransmitter(TelemetryManager panelsTelemetry,
                                            Telemetry dsTelemetry,
                                            String topicPrefix) {
        return JulesBridgeManager.getInstance().getTransmitter(panelsTelemetry, dsTelemetry, topicPrefix);
    }

    public static void advertise(Telemetry telemetry) {
        if (telemetry == null) {
            return;
        }
        telemetry.addLine(advertiseLine());
    }

    public static String advertiseLine() {
        return JulesBridgeManager.getInstance().getAdvertiseLine();
    }

    public static boolean isBridgeOnline() {
        return JulesBridgeManager.getInstance().isRunning();
    }

    public static String token() {
        return JulesBridgeManager.getInstance().getToken();
    }

    public static void stop() {
        JulesBridgeManager.getInstance().stop();
    }
}
