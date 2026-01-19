package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;

@TeleOp(name = "TEST: Grip Reverse (X Button)", group = "Tests")
public class GripReverseTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        BjornHardware hardware = BjornHardware.forTeleOp(hardwareMap);
        
        telemetry.addLine("Grip Reverse Test");
        telemetry.addLine("Press X to reverse grip servos.");
        telemetry.update();
        
        waitForStart();
        
        while (opModeIsActive()) {
            boolean xPressed = gamepad1.x;
            
            if (xPressed) {
                if (hardware.grip1 != null) hardware.grip1.setPower(-1.0);
                if (hardware.grip2 != null) hardware.grip2.setPower(-1.0);
                if (hardware.intake != null) hardware.intake.setPower(-1.0); // Also reverse intake for context
                
                telemetry.addData("Status", "REVERSING (-1.0)");
            } else {
                if (hardware.grip1 != null) hardware.grip1.setPower(0.0);
                if (hardware.grip2 != null) hardware.grip2.setPower(0.0);
                if (hardware.intake != null) hardware.intake.setPower(0.0);
                
                telemetry.addData("Status", "Idle (0.0)");
            }
            
            telemetry.addData("Grip 1 Power", hardware.grip1 != null ? hardware.grip1.getPower() : "NULL");
            telemetry.addData("Grip 2 Power", hardware.grip2 != null ? hardware.grip2.getPower() : "NULL");
            telemetry.update();
        }
    }
}
