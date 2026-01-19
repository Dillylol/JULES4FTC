package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

@TeleOp(name = "IntakeTest", group = "Test")
public class IntakeTest extends LinearOpMode {

    private BjornHardware hardware;

    @Override
    public void runOpMode() {
        hardware = BjornHardware.forTeleOp(hardwareMap);

        telemetry.addLine("Intake Test Ready");
        telemetry.addLine("A: Forward (1.0)");
        telemetry.addLine("B: Reverse (-1.0)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.0;

            if (gamepad1.a) {
                power = 1.0;
            } else if (gamepad1.b) {
                power = -1.0;
            }

            if (hardware.intake != null) {
                hardware.intake.setPower(power);
            }

            telemetry.addData("Intake Power", power);
            telemetry.addLine("A: Forward | B: Reverse");
            telemetry.update();
        }
    }
}
