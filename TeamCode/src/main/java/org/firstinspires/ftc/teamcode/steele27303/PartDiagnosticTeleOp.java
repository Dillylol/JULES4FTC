package org.firstinspires.ftc.teamcode.steele27303;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.BjornHardware;
import org.firstinspires.ftc.teamcode.common.BjornConstants;

@TeleOp(name = "Diagnostic: Hardware Parts", group = "Diagnostic")
public class PartDiagnosticTeleOp extends OpMode {

    private BjornHardware hardware;

    // Test Modes
    private enum Mode {
        DRIVE_MOTORS,
        SUBSYSTEM_MOTORS,
        SERVOS
    }

    private Mode currentMode = Mode.DRIVE_MOTORS;
    private int selectedIndex = 0;
    private boolean dpadUpPrev, dpadDownPrev, dpadLeftPrev, dpadRightPrev;

    // Arrays for selection
    private final String[] driveMotors = { "FL", "FR", "BL", "BR" };
    private final String[] subMotors = { "Intake", "Wheel 1", "Wheel 2", "Turret" };
    private final String[] servos = { "Boot", "Grip 1", "Grip 2", "Brake 1", "Brake 2", "LED 1", "LED 2" };

    @Override
    public void init() {
        hardware = BjornHardware.forTeleOp(hardwareMap);
        telemetry.addLine("Diagnostic Ready.");
        telemetry.addLine("DPAD L/R: Change Mode");
        telemetry.addLine("DPAD U/D: Select Part");
        telemetry.addLine("Triggers: Control Power/Position");
        telemetry.update();
    }

    @Override
    public void loop() {
        handleInput();

        telemetry.addData("Mode", currentMode);

        double power = gamepad1.right_trigger - gamepad1.left_trigger;

        switch (currentMode) {
            case DRIVE_MOTORS:
                testDriveMotors(power);
                break;
            case SUBSYSTEM_MOTORS:
                testSubsystemMotors(power);
                break;
            case SERVOS:
                testServos(power);
                break;
        }

        telemetry.addLine("----------------");
        telemetry.addLine("Use Triggers to Activate");
        telemetry.update();
    }

    private void handleInput() {
        if (gamepad1.dpad_right && !dpadRightPrev) {
            currentMode = Mode.values()[(currentMode.ordinal() + 1) % Mode.values().length];
            selectedIndex = 0;
        }
        if (gamepad1.dpad_left && !dpadLeftPrev) {
            int prev = currentMode.ordinal() - 1;
            if (prev < 0)
                prev = Mode.values().length - 1;
            currentMode = Mode.values()[prev];
            selectedIndex = 0;
        }

        int maxIndex = 0;
        switch (currentMode) {
            case DRIVE_MOTORS:
                maxIndex = driveMotors.length;
                break;
            case SUBSYSTEM_MOTORS:
                maxIndex = subMotors.length;
                break;
            case SERVOS:
                maxIndex = servos.length;
                break;
        }

        if (gamepad1.dpad_down && !dpadDownPrev) {
            selectedIndex = (selectedIndex + 1) % maxIndex;
        }
        if (gamepad1.dpad_up && !dpadUpPrev) {
            selectedIndex--;
            if (selectedIndex < 0)
                selectedIndex = maxIndex - 1;
        }

        dpadUpPrev = gamepad1.dpad_up;
        dpadDownPrev = gamepad1.dpad_down;
        dpadLeftPrev = gamepad1.dpad_left;
        dpadRightPrev = gamepad1.dpad_right;
    }

    private void testDriveMotors(double power) {
        telemetry.addData("Selected", driveMotors[selectedIndex]);
        telemetry.addData("Power", "%.2f", power);

        // Safety: Stop all first
        hardware.frontLeft.setPower(0);
        hardware.frontRight.setPower(0);
        hardware.backLeft.setPower(0);
        hardware.backRight.setPower(0);

        switch (selectedIndex) {
            case 0:
                hardware.frontLeft.setPower(power);
                break;
            case 1:
                hardware.frontRight.setPower(power);
                break;
            case 2:
                hardware.backLeft.setPower(power);
                break;
            case 3:
                hardware.backRight.setPower(power);
                break;
        }
    }

    private void testSubsystemMotors(double power) {
        telemetry.addData("Selected", subMotors[selectedIndex]);
        telemetry.addData("Power", "%.2f", power);

        // Stop all
        if (hardware.intake != null)
            hardware.intake.setPower(0);
        if (hardware.wheel != null)
            hardware.wheel.setPower(0);
        if (hardware.wheel2 != null)
            hardware.wheel2.setPower(0);
        if (hardware.turret != null)
            hardware.turret.setPower(0);

        switch (selectedIndex) {
            case 0:
                if (hardware.intake != null)
                    hardware.intake.setPower(power);
                break;
            case 1:
                if (hardware.wheel != null)
                    hardware.wheel.setPower(power);
                break;
            case 2:
                if (hardware.wheel2 != null)
                    hardware.wheel2.setPower(power);
                break;
            case 3:
                if (hardware.turret != null)
                    hardware.turret.setPower(power);
                break;
        }
    }

    private void testServos(double input) {
        telemetry.addData("Selected", servos[selectedIndex]);

        // Normalize input 0..1 for Position, -1..1 for Power (CR)
        double pos = (input + 1.0) / 2.0; // -1..1 -> 0..1

        switch (selectedIndex) {
            case 0: // Boot
                if (hardware.boot != null) {
                    telemetry.addData("Boot Pos", "%.2f (Triggers -1..1 mapped to 0..1)", pos);
                    hardware.boot.setPosition(pos);
                }
                break;
            case 1: // Grip 1 (CR)
                if (hardware.grip1 != null) {
                    telemetry.addData("Grip 1 Power", "%.2f", input);
                    hardware.grip1.setPower(input);
                }
                break;
            case 2: // Grip 2 (CR)
                if (hardware.grip2 != null) {
                    telemetry.addData("Grip 2 Power", "%.2f", input);
                    hardware.grip2.setPower(input);
                }
                break;
            case 3: // Brake 1
                boolean b1 = input > 0;
                telemetry.addData("Brake 1", b1 ? "ON" : "OFF");
                if (hardware.brake1Red != null)
                    hardware.brake1Red.setState(b1);
                if (hardware.brake1Green != null)
                    hardware.brake1Green.setState(!b1);
                break;
            case 4: // Brake 2
                boolean b2 = input > 0;
                telemetry.addData("Brake 2", b2 ? "ON" : "OFF");
                if (hardware.brake2Red != null)
                    hardware.brake2Red.setState(b2);
                if (hardware.brake2Green != null)
                    hardware.brake2Green.setState(!b2);
                break;
            case 5: // LED 1
                boolean l1 = input > 0;
                telemetry.addData("LED 1", l1 ? "ON" : "OFF");
                if (hardware.led1Red != null)
                    hardware.led1Red.setState(l1); // Red on trigger
                if (hardware.led1Green != null)
                    hardware.led1Green.setState(!l1);
                break;
            case 6: // LED 2
                boolean l2 = input > 0;
                telemetry.addData("LED 2", l2 ? "ON" : "OFF");
                if (hardware.led2Red != null)
                    hardware.led2Red.setState(l2);
                if (hardware.led2Green != null)
                    hardware.led2Green.setState(!l2);
                break;
        }
    }
}
