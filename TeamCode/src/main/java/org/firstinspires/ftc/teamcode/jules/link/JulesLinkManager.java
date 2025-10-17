package org.firstinspires.ftc.teamcode.jules.link;

import android.content.Context;
import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesDataOrganizer;
import org.firstinspires.ftc.teamcode.jules.telemetry.JulesTelemetry;

/**
 * Entry point tying together discovery, scheduling, and transport layers.
 */
public class JulesLinkManager {

    private static final String PREF_WS_URL = "JULES_WS_URL";
    private static final String DEFAULT_WS_URL = "ws://192.168.49.1:8765/stream";

    private final JulesDataOrganizer organizer = JulesDataOrganizer.getInstance();

    private JulesTelemetry telemetryProxy;
    private Telemetry telemetry;
    private JulesWsClient wsClient;
    private JulesUdpBeacon udpBeacon;
    private JulesHttpServer httpServer;
    private JulesScheduler scheduler;
    private String wsUrl;
    private OpMode opMode;

    public void init(OpMode opMode,
                     HardwareMap hardwareMap,
                     Telemetry telemetry,
                     Gamepad gamepad1,
                     Gamepad gamepad2) {
        this.opMode = opMode;
        organizer.bind(opMode, hardwareMap, gamepad1, gamepad2, telemetry);
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.INIT);
        telemetryProxy = new JulesTelemetry(telemetry, organizer);
        this.telemetry = telemetryProxy.getFtcTelemetry();

        Context context = hardwareMap.appContext;
        SharedPreferences prefs = PreferenceManager.getDefaultSharedPreferences(context);
        wsUrl = prefs.getString(PREF_WS_URL, DEFAULT_WS_URL);

        wsClient = new JulesWsClient(wsUrl);
        try {
            udpBeacon = new JulesUdpBeacon();
        } catch (Exception ignored) {
            udpBeacon = null;
        }
        httpServer = new JulesHttpServer(organizer);
        scheduler = new JulesScheduler(organizer, wsClient, udpBeacon, httpServer);
    }

    public void start() {
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.RUNNING);
        wsClient.connect();
        scheduler.start();
    }

    public void loop() {
        // no-op; placeholder for future enhancements
    }

    public void stop() {
        organizer.setOpModeState(JulesDataOrganizer.OpModeState.STOPPED);
        scheduler.stop();
        wsClient.close();
        if (udpBeacon != null) {
            udpBeacon.close();
        }
        if (httpServer != null) {
            httpServer.stopServer();
        }
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public String getWsUrl() {
        return wsUrl;
    }
}

