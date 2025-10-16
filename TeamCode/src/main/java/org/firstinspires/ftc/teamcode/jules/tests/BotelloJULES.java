package org.firstinspires.ftc.teamcode.jules.tests;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.jules.JulesBuilder;
import org.firstinspires.ftc.teamcode.jules.JulesRamTx;
import org.firstinspires.ftc.teamcode.jules.bridge.JulesBridgeManager;

@TeleOp(name = "BotelloJULES")
public class BotelloJULES extends OpMode {

    private DcMotor BackL, BackR, FrontL, FrontR;
    private DcMotorEx Intake, Wheel;
    private IMU imu;

    // Jules Telemetry using the new Builder
    private JulesBuilder jules; // Our new data builder

    @Override
    public void init() {
        // (Hardware mapping and motor setup is the same as before)
        BackL  = hardwareMap.get(DcMotor.class,   "BackL");
        BackR  = hardwareMap.get(DcMotor.class,   "BackR");
        FrontL = hardwareMap.get(DcMotor.class,   "FrontL");
        FrontR = hardwareMap.get(DcMotor.class,   "FrontR");
        Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Wheel  = hardwareMap.get(DcMotorEx.class, "Wheel");
        FrontL.setDirection(DcMotor.Direction.REVERSE);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Wheel.setDirection(DcMotorSimple.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));

        // --- CORRECTED JULES INITIALIZATION ---
        // The JulesBridgeManager handles the web server and gives us the transmitter.
        // We then pass that transmitter to our JulesBuilder.
        // This replaces the manual setup and fixes the error.
        JulesRamTx julesTx = JulesBridgeManager.init(this);
        jules = new JulesBuilder(julesTx);

        telemetry.addData("Jules", "JulesBridgeManager Initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Advertise the Jules server to the Driver Station
        JulesBridgeManager.advertiseToDs(telemetry);

        // (Drive logic is the same as before)
        if (gamepad1.dpad_up) imu.resetYaw();
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX *= 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);
        FrontL.setPower((rotY + rotX + rx) / denominator);
        BackL.setPower((rotY - rotX + rx) / denominator);
        FrontR.setPower((rotY - rotX - rx) / denominator);
        BackR.setPower((rotY + rotX - rx) / denominator);

        // --- Data Logging with JulesBuilder (no changes needed here) ---
        if (jules != null) {
            jules.addData("gamepad_y", y)
                    .addData("gamepad_x", x)
                    .addData("heading_rad", botHeading)
                    .addData("heading_deg", Math.toDegrees(botHeading))
                    .addData("intake_power", Intake.getPower())
                    .addData("wheel_power", Wheel.getPower())
                    .addData("battery_V", getBatteryVoltage())
                    .addData("fl_power", FrontL.getPower())
                    .addData("fr_power", FrontR.getPower())
                    .addData("bl_power", BackL.getPower())
                    .addData("br_power", BackR.getPower());

            jules.send(getRuntime());
        }

        telemetry.addData("Heading", "%.2f deg", Math.toDegrees(botHeading));
        telemetry.update();
    }

    private double getBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor s : hardwareMap.getAll(VoltageSensor.class)) {
            double v = s.getVoltage();
            if (v > 0) min = Math.min(min, v);
        }
        return (min == Double.POSITIVE_INFINITY) ? 0.0 : min;
    }

    @Override
    public void stop() {
        // --- CORRECTED JULES SHUTDOWN ---
        // The manager handles stopping the web server.
        JulesBridgeManager.stop();
    }
}