package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.jules.link.JulesLinkManager;

import java.util.Locale;
import java.util.UUID;
import java.util.concurrent.ThreadLocalRandom;

/**
 * Sample OpMode wiring in the Jules link stack.
 */
@TeleOp(name = "Jules Link Demo", group = "Dev")
public class JulesLinkDemo extends OpMode {

    private enum Mode {
        NORMAL,
        SOAK,
        HANDSHAKE
    }

    private final JulesLinkManager linkManager = new JulesLinkManager();
    private VoltageSensor battery;
    private Mode mode = Mode.NORMAL;
    private boolean modeToggleLatched = false;
    private long lastSoakUpdateMs = 0L;
    private long lastPingMs = 0L;
    private String lastPingId = "";
    private double soakBattery = Double.NaN;
    private double soakMotorPower = 0.0;
    private double soakYaw = 0.0;
    private double soakDistance = 0.0;
    private boolean soakFlag = false;

    @Override
    public void init() {
        linkManager.attachOpMode();
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
        linkManager.onConnected(() -> {
            linkManager.sendSnapshotNow();
            sendHelloFrame();
        });
        sendHelloFrame();
    }

    @Override
    public void loop() {
        linkManager.loop();
        handleModeToggle();
        Telemetry tel = linkManager.getTelemetry();
        tel.addData("Mode", mode);
        double voltage = (battery != null) ? battery.getVoltage() : Double.NaN;
        switch (mode) {
            case SOAK:
                renderSoakTelemetry(tel, voltage);
                break;
            case HANDSHAKE:
                renderHandshakeTelemetry(tel, voltage);
                break;
            case NORMAL:
            default:
                renderNormalTelemetry(tel, voltage);
                break;
        }
        tel.update();
    }

    @Override
    public void stop() {
        linkManager.stop();
        linkManager.detachOpMode();
    }

    private void handleModeToggle() {
        boolean pressed = gamepad1 != null && gamepad1.right_bumper;
        if (pressed && !modeToggleLatched) {
            int nextOrdinal = (mode.ordinal() + 1) % Mode.values().length;
            mode = Mode.values()[nextOrdinal];
            if (mode != Mode.SOAK) {
                lastSoakUpdateMs = 0L;
            }
            if (mode != Mode.HANDSHAKE) {
                lastPingMs = 0L;
                lastPingId = "";
            }
        }
        modeToggleLatched = pressed;
    }

    private void renderNormalTelemetry(Telemetry tel, double voltage) {
        tel.addData("Battery", Double.isNaN(voltage) ? "n/a" : String.format(Locale.US, "%.2f V", voltage));
        tel.addData("WS", linkManager.getWsUrl());
    }

    private void renderSoakTelemetry(Telemetry tel, double voltage) {
        long now = System.currentTimeMillis();
        if (now - lastSoakUpdateMs >= 75L) {
            lastSoakUpdateMs = now;
            ThreadLocalRandom rand = ThreadLocalRandom.current();
            soakBattery = 11.0 + rand.nextDouble(2.5);
            soakMotorPower = rand.nextDouble(-1.0, 1.0);
            soakYaw = rand.nextDouble(-180.0, 180.0);
            soakDistance = rand.nextDouble(50.0, 1500.0);
            soakFlag = rand.nextBoolean();
        }
        tel.addData("Battery", Double.isNaN(voltage) ? "n/a" : String.format(Locale.US, "%.2f V", voltage));
        tel.addData("WS", linkManager.getWsUrl());
        tel.addData("SoakBattery", String.format(Locale.US, "%.2f V", soakBattery));
        tel.addData("SoakMotorA.Power", String.format(Locale.US, "%.2f", soakMotorPower));
        tel.addData("SoakIMU.Yaw", String.format(Locale.US, "%.1f", soakYaw));
        tel.addData("SoakDist.Front.mm", String.format(Locale.US, "%.0f", soakDistance));
        tel.addData("SoakFlag", soakFlag);
    }

    private void renderHandshakeTelemetry(Telemetry tel, double voltage) {
        long now = System.currentTimeMillis();
        if (now - lastPingMs >= 2000L) {
            lastPingMs = now;
            lastPingId = UUID.randomUUID().toString();
            JsonObject ping = new JsonObject();
            ping.addProperty("type", "ping");
            ping.addProperty("id", lastPingId);
            ping.addProperty("t0", now);
            linkManager.sendNdjson(ping.toString());
        }
        tel.addData("Battery", Double.isNaN(voltage) ? "n/a" : String.format(Locale.US, "%.2f V", voltage));
        tel.addData("WS", linkManager.getWsUrl());
        if (!lastPingId.isEmpty()) {
            tel.addData("Handshake.PingId", lastPingId);
            tel.addData("Handshake.LastPingMs", lastPingMs);
        }
    }

    private void sendHelloFrame() {
        JsonObject hello = new JsonObject();
        hello.addProperty("type", "hello");
        hello.addProperty("proto", "jules/1");
        hello.addProperty("server_time_ms", System.currentTimeMillis());
        linkManager.sendNdjson(hello.toString());
    }
}

