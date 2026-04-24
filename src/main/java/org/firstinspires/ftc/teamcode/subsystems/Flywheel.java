package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public final class Flywheel {
    public static double kP = 0.005;
    public static double kS = 0.1;
    public static double kV = 0.00033;
    public static int velocityTolerance = 25;
    private final DcMotorEx flywheelMotorTop;
    private final DcMotorEx flywheelMotorBottom;
    private final Context context;
    private Mode mode = Mode.OFF;
    private double target = 0;

    public Flywheel(Context context) {
        this.context = context;
        flywheelMotorTop = context.motor("flywheelTop");
        flywheelMotorBottom = context.motor("flywheelBottom");
        flywheelMotorBottom.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void off() {
        mode = Mode.OFF;
    }

    public void on() {
        mode = Mode.VELOCITY;
    }

    public void toggle() {
        mode = mode == Mode.OFF ? Mode.VELOCITY : Mode.OFF;
    }

    private void setPower(double power) {
        flywheelMotorTop.setPower(power);
        flywheelMotorBottom.setPower(power);
    }

    public double getVelocity() {
        return flywheelMotorBottom.getVelocity();
    }
    public double getTarget() {
        return target;
    }

    public boolean atTarget() {
        return Math.abs(target - getVelocity()) <= velocityTolerance;
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case OFF:
                    setPower(0);
                    return;
                case VELOCITY:
                    setPower(kP * (target - getVelocity()) + kV * target + kS * Math.signum(target));
                    break;
            }

            context.telemetry.addData("Flywheel/velocity", getVelocity());
            context.telemetry.addData("Flywheel/target", target);
            context.telemetry.addData("Flywheel/power", flywheelMotorBottom.getPower());
        });
    }

    enum Mode {
        OFF,
        VELOCITY
    }
}
