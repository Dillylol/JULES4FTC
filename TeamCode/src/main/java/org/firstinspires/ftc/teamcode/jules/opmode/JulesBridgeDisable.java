package org.firstinspires.ftc.teamcode.jules.opmode;

import android.content.Context;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

@TeleOp(name = "JULES: Disable", group = "Jules")
public class JulesBridgeDisable extends OpMode {

    private JulesBridgeManager manager;
    private Telemetry panelsTl;

    @Override
    public void init() {
        manager = JulesBridgeManager.getInstance();
        Context appContext = hardwareMap.appContext;
        manager.prepare(appContext);
        panelsTl = PanelsTelemetry.INSTANCE.getFtcTelemetry();

        telemetry.addLine("Press START to stop the JULES bridge.");
        telemetry.update();

        if (panelsTl != null) {
            panelsTl.addLine("Press START to stop the JULES bridge.");
            panelsTl.update();
        }
    }

    @Override
    public void start() {
        manager.stop();
        manager.setAutoEnabled(false);
        emitStatus("Bridge stop requested.");
    }

    @Override
    public void loop() {
        emitStatus("Bridge is now OFF.");
    }

    @Override
    public void stop() {
        emitStatus("Bridge remains OFF.");
    }

    private void emitStatus(String headline) {
        JulesBridgeManager.Status status = manager.getStatusSnapshot();
        String state = (status != null && status.running) ? "RUNNING" : "STOPPED";
        String masked = (status != null) ? status.maskedToken : JulesBridgeManager.maskToken(null);
        String token = (status != null) ? status.token : null;
        String advertise = (status != null) ? status.advertiseLine : manager.getAdvertiseLine();

        telemetry.addLine(headline);
        telemetry.addData("Status", state);
        telemetry.addData("Token", masked);
        telemetry.addLine(advertise != null ? advertise : "");
        telemetry.update();

        if (panelsTl != null) {
            panelsTl.addLine(headline);
            panelsTl.addData("Status", state);
            panelsTl.addData("Token", token != null ? token : "n/a");
            panelsTl.addLine(advertise != null ? advertise : "");
            panelsTl.update();
        }
    }
}
