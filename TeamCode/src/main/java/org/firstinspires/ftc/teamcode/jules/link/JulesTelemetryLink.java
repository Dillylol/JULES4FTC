package org.firstinspires.ftc.teamcode.jules.link;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.link.JulesWsLink;

@TeleOp(name="JulesTelemetryLink", group="Dev")
public class JulesTelemetryLink extends OpMode {
    private JulesWsLink link;
    private VoltageSensor battery;
    private Telemetry tel;

    // CHANGE THIS to your laptop IP on the robot network
    private static final String WS_URL = "ws://192.168.49.1:8765/stream";

    @Override public void init() {
        tel = telemetry;
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
        link = new JulesWsLink(WS_URL, tel);
        link.connect();
    }

    @Override public void loop() {
        link.sendHeartbeat(this, battery); // 5 Hz scheduler also runs; this guarantees at least once per loop
        // ... your existing drive code ...
        tel.addData("Battery (V)", battery != null ? battery.getVoltage() : "N/A");
        tel.update();
    }

    @Override public void stop() {
        if (link != null) link.close();
    }
}
