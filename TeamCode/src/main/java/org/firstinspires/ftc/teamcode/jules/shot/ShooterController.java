package org.firstinspires.ftc.teamcode.jules.shot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.BjornConstants;

/**
 * Owns the flywheel, intake, and lift hardware for a single-shot cycle.
 */
public final class ShooterController {
    private static final double WHEEL_TPR = 28.0; // 1:1 goBILDA 5202
    private static final double INTAKE_PULSE_POWER = 1.0;
    private static final long INTAKE_PULSE_MS = 200;

    private final DcMotorEx flywheel;
    private final DcMotorEx intake;
    private final Servo lift;

    private int targetRpm;
    private long intakePulseUntilMs = -1;

    public ShooterController(DcMotorEx flywheel, DcMotorEx intake, Servo lift) {
        this.flywheel = flywheel;
        this.intake = intake;
        this.lift = lift;

        if (this.flywheel != null) {
            try {
                this.flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } catch (Exception ignored) {
            }
        }
        if (this.intake != null) {
            try {
                this.intake.setPower(0.0);
            } catch (Exception ignored) {
            }
        }
        closeLift();
        targetRpm = 0;
    }

    public void setFlywheelTarget(int rpm) {
        int clamped = Math.max(0, rpm);
        if (clamped == targetRpm) {
            return;
        }
        targetRpm = clamped;
        if (flywheel == null) {
            return;
        }
        try {
            if (clamped > 0) {
                double tps = (clamped / 60.0) * WHEEL_TPR;
                if (flywheel.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                flywheel.setVelocity(tps);
            } else {
                flywheel.setPower(0.0);
            }
        } catch (Exception ignored) {
        }
    }

    public int getTargetRpm() {
        return targetRpm;
    }

    public double getMeasuredRpm() {
        if (flywheel == null) {
            return 0.0;
        }
        try {
            double tps = flywheel.getVelocity();
            return (tps / WHEEL_TPR) * 60.0;
        } catch (Exception e) {
            return 0.0;
        }
    }

    public boolean isReady(int tolRpm) {
        if (targetRpm <= 0) {
            return false;
        }
        double error = Math.abs(getMeasuredRpm() - targetRpm);
        return error <= tolRpm;
    }

    public void openLift() {
        if (lift != null) {
            try {
                lift.setPosition(BjornConstants.Servos.LIFT_RAISED);
            } catch (Exception ignored) {
            }
        }
    }

    public void closeLift() {
        if (lift != null) {
            try {
                lift.setPosition(BjornConstants.Servos.LIFT_LOWERED);
            } catch (Exception ignored) {
            }
        }
    }

    public void pulseIntake(long nowMs) {
        if (intake != null) {
            try {
                intake.setPower(INTAKE_PULSE_POWER);
                intakePulseUntilMs = nowMs + INTAKE_PULSE_MS;
            } catch (Exception ignored) {
            }
        }
    }

    public void stopIntake() {
        intakePulseUntilMs = -1;
        if (intake != null) {
            try {
                intake.setPower(0.0);
            } catch (Exception ignored) {
            }
        }
    }

    public void update(long nowMs) {
        if (intakePulseUntilMs > 0 && nowMs >= intakePulseUntilMs) {
            stopIntake();
        }
    }

    public void reset() {
        setFlywheelTarget(0);
        stopIntake();
        closeLift();
    }
}
