package org.firstinspires.ftc.teamcode.jules;

import android.content.Context;

import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;

/**
 * Compatibility façade that delegates to {@link JulesBridgeManager}.
 *
 * <p>Existing OpModes can continue calling {@code JulesService} helpers while the
 * lifecycle is owned by the singleton bridge manager.</p>
 */
public final class JulesService {

    private static final JulesBridgeManager MANAGER = JulesBridgeManager.getInstance();

    private JulesService() { }

    /**
     * Prepare the bridge manager with the application context. No network I/O is performed here.
     */
    public static void init(Context context) {
        MANAGER.prepare(context);
    }

    public static JulesBuilder newBuilder(TelemetryManager panelsTelemetry,
                                          Telemetry dsTelemetry,
                                          String topicPrefix) {
        return MANAGER.newBuilder(panelsTelemetry, dsTelemetry, topicPrefix);
    }

    public static JulesRamTx newTransmitter(TelemetryManager panelsTelemetry,
                                            Telemetry dsTelemetry,
                                            String topicPrefix) {
        return MANAGER.getTransmitter(panelsTelemetry, dsTelemetry, topicPrefix);
    }

    public static void advertise(Telemetry telemetry) {
        if (telemetry == null) {
            return;
        }
        telemetry.addLine(advertiseLine());
    }

    public static String advertiseLine() {
        return MANAGER.getAdvertiseLine();
    }

    public static boolean isBridgeOnline() {
        return MANAGER.isRunning();
    }

    public static String token() {
        return MANAGER.getToken();
    }

    public static void stop() {
        MANAGER.stop();
    }
}
