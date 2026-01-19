package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends LinearOpMode {

    private BjornHardware hardware;

    @Override
    public void runOpMode() {
        hardware = BjornHardware.forTeleOp(hardwareMap);

        telemetry.addLine("Servo Test Ready");
        telemetry.addLine("A: Grip Forward");
        telemetry.addLine("B: Grip Reverse");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double power = 0.0;

            if (gamepad1.a) {
                power = 1.0;
            } else if (gamepad1.b) {
                power = -1.0;
            }

            // Both grips move in the same logical direction (as defined in constants/hardware)
            if (hardware.grip1 != null) {
                hardware.grip1.setPower(power);
            }
            if (hardware.grip2 != null) {
                hardware.grip2.setPower(power);
            }

            telemetry.addData("Grip Power", power);
            telemetry.addLine("A: Fwd | B: Rev");
            telemetry.update();
        }
    }
}
