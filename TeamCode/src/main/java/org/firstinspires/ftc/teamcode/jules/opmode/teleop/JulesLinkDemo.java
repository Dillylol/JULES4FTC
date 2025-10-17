package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.link.JulesLinkManager;

import java.util.Locale;

/**
 * Sample OpMode wiring in the Jules link stack.
 */
@TeleOp(name = "Jules Link Demo", group = "Dev")
public class JulesLinkDemo extends OpMode {

    private final JulesLinkManager linkManager = new JulesLinkManager();
    private VoltageSensor battery;

    @Override
    public void init() {
        linkManager.init(this, hardwareMap, telemetry, gamepad1, gamepad2);
        this.telemetry = linkManager.getTelemetry();
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next()
                : null;
        telemetry.addData("JULES", "Telemetry link initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        linkManager.start();
    }

    @Override
    public void loop() {
        linkManager.loop();
        Telemetry tel = linkManager.getTelemetry();
        double voltage = (battery != null) ? battery.getVoltage() : Double.NaN;
        tel.addData("Battery", Double.isNaN(voltage) ? "n/a" : String.format(Locale.US, "%.2f V", voltage));
        tel.addData("WS", linkManager.getWsUrl());
        tel.update();
    }

    @Override
    public void stop() {
        linkManager.stop();
    }
}

