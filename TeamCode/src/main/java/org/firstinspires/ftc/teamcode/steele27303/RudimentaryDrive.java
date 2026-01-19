package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Rudimentary Drive Program for Hardware Testing.
 * - Standalone hardware definition (No BjornHardware).
 * - "Forward" wheels (Front Left / Front Right) are REVERSED as requested.
 * - Controls: Left Stick (Drive), Right Stick (Turn).
 */
@TeleOp(name = "RudimentaryDrive", group = "Test")
public class RudimentaryDrive extends LinearOpMode {

    private DcMotor fl, fr, bl, br;

    @Override
    public void runOpMode() {
        // 1. Standalone Hardware Init
        fl = hardwareMap.get(DcMotor.class, "lf"); // Assuming standard config names
        fr = hardwareMap.get(DcMotor.class, "rf");
        bl = hardwareMap.get(DcMotor.class, "lr");
        br = hardwareMap.get(DcMotor.class, "rr");

        // 2. Set Directions (Reverse Forward Wheels L&R)
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        // Back wheels default to FORWARD
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        // Zero Power Behavior
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Rudimentary Drive Ready");
        telemetry.addLine("Front Wheels Reversed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed (Up is negative)
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            
            // Standard Mecanum math
            double flPower = (y + x + rx) / denominator;
            double blPower = (y - x + rx) / denominator;
            double frPower = (y - x - rx) / denominator;
            double brPower = (y + x - rx) / denominator;

            fl.setPower(flPower);
            bl.setPower(blPower);
            fr.setPower(frPower);
            br.setPower(brPower);

            telemetry.addData("FL Power", "%.2f", flPower);
            telemetry.addData("FR Power", "%.2f", frPower);
            telemetry.addData("BL Power", "%.2f", blPower);
            telemetry.addData("BR Power", "%.2f", brPower);
            telemetry.update();
        }
    }
}
