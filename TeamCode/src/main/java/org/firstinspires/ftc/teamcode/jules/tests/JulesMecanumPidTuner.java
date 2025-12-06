package org.firstinspires.ftc.teamcode.jules.tests;

import com.google.gson.JsonObject;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.jules.core.JulesRobot;

@Autonomous(name = "JULES: Mecanum PID Tuner", group = "JULES")
public class JulesMecanumPidTuner extends OpMode {

    private JulesRobot robot;
    private long startTime;
    private static final double TEST_POWER = 0.5;
    private static final int TEST_DURATION_MS = 2000;

    @Override
    public void init() {
        robot = new JulesRobot(hardwareMap, telemetry);
        robot.init();
        
        // Ensure encoders are reset
        resetEncoders(robot.leftFront);
        resetEncoders(robot.leftRear);
        resetEncoders(robot.rightFront);
        resetEncoders(robot.rightRear);
        
        telemetry.addLine("Ready to Tune. Press Start.");
    }
    
    private void resetEncoders(DcMotor m) {
        if (m != null) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
        robot.setDrivePowers(TEST_POWER, TEST_POWER, TEST_POWER, TEST_POWER);
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();
        long elapsed = now - startTime;

        if (elapsed < TEST_DURATION_MS) {
            // Report Velocity
            JsonObject data = new JsonObject();
            data.addProperty("type", "pid_data");
            data.addProperty("ts", elapsed);
            data.addProperty("lf_v", ((DcMotorEx)robot.leftFront).getVelocity());
            data.addProperty("lr_v", ((DcMotorEx)robot.leftRear).getVelocity());
            data.addProperty("rf_v", ((DcMotorEx)robot.rightFront).getVelocity());
            data.addProperty("rr_v", ((DcMotorEx)robot.rightRear).getVelocity());
            robot.publish(data.toString());
        } else {
            robot.setDrivePowers(0, 0, 0, 0);
            telemetry.addLine("Test Complete.");
        }
        
        telemetry.addData("Time", elapsed);
        telemetry.update();
    }
}
