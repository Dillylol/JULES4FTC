package org.firstinspires.ftc.teamcode.common.turret;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Gear Ratio Test - Encoder to Degree Validation (STANDALONE RIG VERSION)
 * 
 * For testing on a rig with only: turret motor, controller
 * No drive motors or IMU required.
 * 
 * Validates the turret gear ratio calculation by comparing
 * raw encoder ticks to calculated degrees.
 * 
 * Expected: 80:1 total gear reduction (5:1 * 4:1 * 4:1)
 * Motor: 28 ticks/rev (REV HD Hex)
 * TURRET_TICKS_PER_DEGREE = (28 * 80) / 360 = 6.222 ticks/deg
 * 
 * Controls:
 * - D-Pad Left/Right: Manual turret control
 * - Y: Reset encoder to 0
 * - X: Mark reference position
 * - A: Toggle continuous rotation test mode
 */
@TeleOp(name = "TEST: Gear Ratio (Rig)", group = "Tests")
public class GearRatioTest extends LinearOpMode {

    // Hardware - defined directly for standalone rig testing
    private DcMotorEx turret;

    // Constants
    private static final double MOTOR_TICKS_PER_REV = 28.0;
    private static final double TURRET_GEAR_REDUCTION = 5.0 * 4.0 * 4.0; // 80:1
    private static final double TURRET_TICKS_PER_DEGREE = (MOTOR_TICKS_PER_REV * TURRET_GEAR_REDUCTION) / 360.0;

    private static final double TEST_POWER = 0.25; // Slow for accuracy

    // State
    private int referencePosition = 0;
    private boolean referenceSet = false;
    private boolean continuousMode = false;

    private boolean yPrev = false;
    private boolean xPrev = false;
    private boolean aPrev = false;

    @Override
    public void runOpMode() {
        // Initialize turret motor directly
        turret = hardwareMap.get(DcMotorEx.class, "Turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Gear Ratio Test (Rig)");
        telemetry.addLine("D-Pad: Manual turret control");
        telemetry.addLine("Y: Reset encoder");
        telemetry.addLine("X: Mark reference position");
        telemetry.addLine("A: Toggle continuous test");
        telemetry.addLine();
        telemetry.addData("Expected Ticks/Deg", "%.3f", TURRET_TICKS_PER_DEGREE);
        telemetry.addData("Gear Ratio", "%.0f:1", TURRET_GEAR_REDUCTION);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Raw encoder position
            int rawTicks = turret.getCurrentPosition();

            // Calculated degrees using the formula
            double calculatedDegrees = rawTicks / TURRET_TICKS_PER_DEGREE;

            // Input handling
            if (gamepad1.y && !yPrev) {
                // Reset encoder
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                referenceSet = false;
            }
            yPrev = gamepad1.y;

            if (gamepad1.x && !xPrev) {
                referencePosition = rawTicks;
                referenceSet = true;
            }
            xPrev = gamepad1.x;

            if (gamepad1.a && !aPrev) {
                continuousMode = !continuousMode;
            }
            aPrev = gamepad1.a;

            // Motor control
            double power = 0.0;
            if (continuousMode) {
                // Slow continuous rotation for testing
                power = TEST_POWER;
            } else {
                // Manual D-Pad control
                if (gamepad1.dpad_left) {
                    power = TEST_POWER;
                } else if (gamepad1.dpad_right) {
                    power = -TEST_POWER;
                }
            }
            turret.setPower(power);

            // Delta from reference
            int deltaFromRef = rawTicks - referencePosition;
            double deltaDegreesFromRef = deltaFromRef / TURRET_TICKS_PER_DEGREE;

            // Telemetry
            telemetry.addLine("=== Gear Ratio Test (Rig) ===");
            telemetry.addData("Mode", continuousMode ? "CONTINUOUS" : "MANUAL");
            telemetry.addLine();

            telemetry.addLine("--- Raw Values ---");
            telemetry.addData("Encoder Ticks", rawTicks);
            telemetry.addData("Calculated Degrees", "%.2f°", calculatedDegrees);
            telemetry.addLine();

            telemetry.addLine("--- Constants ---");
            telemetry.addData("Ticks/Degree", "%.4f", TURRET_TICKS_PER_DEGREE);
            telemetry.addData("Gear Reduction", "%.0f:1", TURRET_GEAR_REDUCTION);
            telemetry.addLine();

            if (referenceSet) {
                telemetry.addLine("--- From Reference ---");
                telemetry.addData("Ref Position", referencePosition);
                telemetry.addData("Delta Ticks", deltaFromRef);
                telemetry.addData("Delta Degrees", "%.2f°", deltaDegreesFromRef);
            } else {
                telemetry.addLine("Press X to set reference");
            }

            telemetry.addLine();
            telemetry.addLine("--- Validation ---");
            telemetry.addData("90° Should Be", "%.0f ticks", 90.0 * TURRET_TICKS_PER_DEGREE);
            telemetry.addData("180° Should Be", "%.0f ticks", 180.0 * TURRET_TICKS_PER_DEGREE);

            telemetry.update();
        }
    }
}
