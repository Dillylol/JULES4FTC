package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

@TeleOp(name = "JULES: Handshake", group = "JULES")
public class JulesHandshake extends OpMode {

    private JulesRobot robot;
    private long lastSend = 0;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        telemetry.addLine("JULES Handshake Ready");
    }

    @Override
    public void start() {
        robot.start();
    }

    @Override
    public void loop() {
        robot.update();

        long now = System.currentTimeMillis();
        if (now - lastSend > 1000) {
            JsonObject msg = new JsonObject();
            msg.addProperty("type", "handshake");
            msg.addProperty("uptime", getRuntime());
            msg.addProperty("battery", robot.batterySensor != null ? robot.batterySensor.getVoltage() : 0);
            robot.publish(msg.toString());
            lastSend = now;
            telemetry.addLine("Sent Handshake...");
        }
        
        telemetry.update();
    }
}
