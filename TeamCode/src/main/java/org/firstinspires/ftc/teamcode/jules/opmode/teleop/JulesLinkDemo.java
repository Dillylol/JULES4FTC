package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.google.gson.JsonObject;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Random;
import java.util.UUID;

import org.firstinspires.ftc.teamcode.jules.link.JulesLinkManager;

/**
 * Sample OpMode showing how to enable the Jules link manager.
 */
@TeleOp(name = "Jules Link Demo", group = "JULES")
public final class JulesLinkDemo extends OpMode {

    private enum Mode {
        NORMAL,
        SOAK,
        HANDSHAKE;

        Mode next() {
            return values()[(ordinal() + 1) % values().length];
        }
    }

    private static final double SOAK_INTERVAL_MS = 75.0;
    private static final double HANDSHAKE_INTERVAL_MS = 2000.0;

    private final JulesLinkManager manager = new JulesLinkManager();
    private final ElapsedTime soakTimer = new ElapsedTime();
    private final ElapsedTime pingTimer = new ElapsedTime();
    private final Random random = new Random();

    private Mode mode = Mode.NORMAL;
    private boolean modeToggleLatched;
    private double soakBattery;
    private double soakMotorPower;
    private int soakMotorPosition;
    private double soakYaw;
    private int soakDistance;
    private boolean soakFlag;
    private String lastPingId = "";
    private long lastPingTimeMs;

    @Override
    public void init() {
        manager.attachOpMode();
        manager.init(this, hardwareMap, telemetry, gamepad1, gamepad2);
        telemetry.addLine("Jules link ready");
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
        mode = Mode.NORMAL;
        modeToggleLatched = false;
        soakTimer.reset();
        pingTimer.reset();
        manager.onConnected(() -> {
            manager.sendSnapshotNow();
            if (mode == Mode.HANDSHAKE) {
                sendHelloFrame();
                sendPingFrame();
            }
        });
        manager.start();
        manager.sendSnapshotNow();
        sendHelloFrame();
        linkManager.start();
    }

    @Override
    public void loop() {
        manager.loop();
        updateModeToggle();
        pushModeTelemetry();
        telemetry.addData("runtime", String.format("%.2f", getRuntime()));
        telemetry.addData("mode", mode);
        telemetry.update();
        linkManager.loop();
        Telemetry tel = linkManager.getTelemetry();
        double voltage = (battery != null) ? battery.getVoltage() : Double.NaN;
        tel.addData("Battery", Double.isNaN(voltage) ? "n/a" : String.format(Locale.US, "%.2f V", voltage));
        tel.addData("WS", linkManager.getWsUrl());
        tel.update();
    }

    @Override
    public void stop() {
        manager.detachOpMode();
    }

    private void updateModeToggle() {
        boolean pressed = gamepad1 != null && gamepad1.x;
        if (pressed && !modeToggleLatched) {
            Mode next = mode.next();
            onModeChanged(next);
            mode = next;
            modeToggleLatched = true;
        } else if (!pressed) {
            modeToggleLatched = false;
        }
    }

    private void onModeChanged(Mode next) {
        if (next == Mode.SOAK) {
            soakTimer.reset();
            generateSoakValues();
        } else if (next == Mode.HANDSHAKE) {
            pingTimer.reset();
            sendHelloFrame();
            sendPingFrame();
        }
    }

    private void pushModeTelemetry() {
        if (mode == Mode.SOAK) {
            if (soakTimer.milliseconds() >= SOAK_INTERVAL_MS) {
                soakTimer.reset();
                generateSoakValues();
            }
            telemetry.addData("SoakBattery", String.format("%.2f", soakBattery));
            telemetry.addData("SoakMotorA.Power", soakMotorPower);
            telemetry.addData("SoakMotorA.Position", soakMotorPosition);
            telemetry.addData("SoakIMU.Yaw", soakYaw);
            telemetry.addData("SoakDist.Front.mm", soakDistance);
            telemetry.addData("SoakFlag", soakFlag);
        } else if (mode == Mode.HANDSHAKE) {
            if (pingTimer.milliseconds() >= HANDSHAKE_INTERVAL_MS) {
                pingTimer.reset();
                sendPingFrame();
            }
            telemetry.addData("HandshakePingId", lastPingId);
            telemetry.addData("HandshakePingTimeMs", lastPingTimeMs);
        }
    }

    private void generateSoakValues() {
        soakBattery = 10.0 + random.nextDouble() * 3.0;
        soakMotorPower = random.nextDouble() * 2.0 - 1.0;
        soakMotorPosition = random.nextInt(10000);
        soakYaw = random.nextDouble() * 360.0 - 180.0;
        soakDistance = 100 + random.nextInt(1500);
        soakFlag = random.nextBoolean();
    }

    private void sendHelloFrame() {
        JsonObject hello = new JsonObject();
        hello.addProperty("type", "hello");
        hello.addProperty("proto", "jules/1");
        hello.addProperty("server_time_ms", System.currentTimeMillis());
        manager.sendNdjson(hello.toString());
    }

    private void sendPingFrame() {
        JsonObject ping = new JsonObject();
        ping.addProperty("type", "ping");
        String id = UUID.randomUUID().toString();
        long now = System.currentTimeMillis();
        ping.addProperty("id", id);
        ping.addProperty("t0", now);
        manager.sendNdjson(ping.toString());
        lastPingId = id;
        lastPingTimeMs = now;
        linkManager.stop();
    }
}

