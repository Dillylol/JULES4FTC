package org.firstinspires.ftc.teamcode.jules.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.jules.constants.JulesConstants;

/**
 * Motor Tester: use D-pad to select a motor, bumpers to spin it.
 *
 * D-pad Up = select Front Left
 * D-pad Right = select Front Right
 * D-pad Down = select Back Left (down-left quadrant)
 * D-pad Left = select Back Right (down-right quadrant... using left as "other
 * back")
 *
 * Right Bumper = spin selected motor FORWARD at 0.3
 * Left Bumper = spin selected motor REVERSE at 0.3
 * Release both = stop selected motor
 *
 * A button = spin ALL motors forward (verify drive-straight)
 * B button = stop ALL motors
 */
@TeleOp(name = "JULES Motor Tester", group = "Jules")
public class JulesMotorTester extends OpMode {

    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private String selectedName = "leftFront";
    private DcMotor selectedMotor;
    private boolean prevUp, prevRight, prevDown, prevLeft;
    private static final double TEST_POWER = 0.3;

    @Override
    public void init() {
        leftFront = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.FRONT_LEFT);
        leftRear = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.BACK_LEFT);
        rightFront = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.FRONT_RIGHT);
        rightRear = hardwareMap.tryGet(DcMotor.class, JulesConstants.Motors.BACK_RIGHT);

        // Apply configured directions
        if (leftFront != null)
            leftFront.setDirection(JulesConstants.Motors.FRONT_LEFT_DIR);
        if (leftRear != null)
            leftRear.setDirection(JulesConstants.Motors.BACK_LEFT_DIR);
        if (rightFront != null)
            rightFront.setDirection(JulesConstants.Motors.FRONT_RIGHT_DIR);
        if (rightRear != null)
            rightRear.setDirection(JulesConstants.Motors.BACK_RIGHT_DIR);

        selectedMotor = leftFront;
        telemetry.addData("Status", "Ready — use D-pad to select motor");
    }

    @Override
    public void loop() {
        // --- D-pad rising-edge selection ---
        if (gamepad1.dpad_up && !prevUp) {
            selectedName = "leftFront (" + JulesConstants.Motors.FRONT_LEFT + ")";
            selectedMotor = leftFront;
            stopAll();
        }
        if (gamepad1.dpad_right && !prevRight) {
            selectedName = "rightFront (" + JulesConstants.Motors.FRONT_RIGHT + ")";
            selectedMotor = rightFront;
            stopAll();
        }
        if (gamepad1.dpad_down && !prevDown) {
            selectedName = "leftRear (" + JulesConstants.Motors.BACK_LEFT + ")";
            selectedMotor = leftRear;
            stopAll();
        }
        if (gamepad1.dpad_left && !prevLeft) {
            selectedName = "rightRear (" + JulesConstants.Motors.BACK_RIGHT + ")";
            selectedMotor = rightRear;
            stopAll();
        }
        prevUp = gamepad1.dpad_up;
        prevRight = gamepad1.dpad_right;
        prevDown = gamepad1.dpad_down;
        prevLeft = gamepad1.dpad_left;

        // --- Bumpers spin the selected motor ---
        if (selectedMotor != null) {
            if (gamepad1.right_bumper) {
                selectedMotor.setPower(TEST_POWER);
            } else if (gamepad1.left_bumper) {
                selectedMotor.setPower(-TEST_POWER);
            } else {
                selectedMotor.setPower(0);
            }
        }

        // --- A = all forward, B = stop all ---
        if (gamepad1.a) {
            setPowerAll(TEST_POWER);
        }
        if (gamepad1.b) {
            stopAll();
        }

        // --- Telemetry ---
        telemetry.addData("Selected", selectedName);
        telemetry.addData("Motor Found", selectedMotor != null ? "YES" : "NO");
        telemetry.addData("Power", selectedMotor != null ? String.format("%.2f", selectedMotor.getPower()) : "N/A");
        telemetry.addLine();
        telemetry.addData("Controls", "D-pad=select | RB=fwd | LB=rev");
        telemetry.addData("         ", "A=all fwd | B=stop all");
        telemetry.addLine();
        telemetry.addData("LF (" + JulesConstants.Motors.FRONT_LEFT + ")",
                leftFront != null ? String.format("%.2f  dir=%s", leftFront.getPower(), leftFront.getDirection())
                        : "NULL");
        telemetry.addData("RF (" + JulesConstants.Motors.FRONT_RIGHT + ")",
                rightFront != null ? String.format("%.2f  dir=%s", rightFront.getPower(), rightFront.getDirection())
                        : "NULL");
        telemetry.addData("LR (" + JulesConstants.Motors.BACK_LEFT + ")",
                leftRear != null ? String.format("%.2f  dir=%s", leftRear.getPower(), leftRear.getDirection())
                        : "NULL");
        telemetry.addData("RR (" + JulesConstants.Motors.BACK_RIGHT + ")",
                rightRear != null ? String.format("%.2f  dir=%s", rightRear.getPower(), rightRear.getDirection())
                        : "NULL");
    }

    private void stopAll() {
        if (leftFront != null)
            leftFront.setPower(0);
        if (leftRear != null)
            leftRear.setPower(0);
        if (rightFront != null)
            rightFront.setPower(0);
        if (rightRear != null)
            rightRear.setPower(0);
    }

    private void setPowerAll(double power) {
        if (leftFront != null)
            leftFront.setPower(power);
        if (leftRear != null)
            leftRear.setPower(power);
        if (rightFront != null)
            rightFront.setPower(power);
        if (rightRear != null)
            rightRear.setPower(power);
    }

    @Override
    public void stop() {
        stopAll();
    }
}
