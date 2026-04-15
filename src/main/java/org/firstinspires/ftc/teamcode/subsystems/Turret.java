package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public final class Turret {
    private static final double TICKS_PER_REVOLUTION = 8192;
    public static double minimumAngleDegrees = -90;
    public static double incrementDegrees = 2.5;
    public static PIDFCoefficients coefficients = new PIDFCoefficients(0.02567, 0, 0.000867, 0);
    private static double angleTransfer = 0;
    private final DcMotorEx turretMotor;
    private final Context context;
    private final PIDFController controller = new PIDFController(coefficients);
    private double targetDegrees = 0;
    private Mode mode = Mode.OFF;
    private double angleOffsetDegrees = 0;

    public Turret(Context context) {
        this.context = context;
        turretMotor = context.hardwareMap.get(DcMotorEx.class, "turret");

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static void localize(double angle) {
        angleTransfer = angle;
    }

    private static double legalizeAngle(double angle) {
        return ((angle - minimumAngleDegrees) % 360 + 360) % 360 + minimumAngleDegrees;
    }

    private double getRawAngleDegrees() {
        return turretMotor.getCurrentPosition() / TICKS_PER_REVOLUTION * 360;
    }

    public double getAngleDegrees() {
        return getRawAngleDegrees() + angleOffsetDegrees;
    }

    private void setAngleDegrees(double angle) {
        angleOffsetDegrees = angle - getRawAngleDegrees();
    }

    public void off() {
        mode = Mode.OFF;
    }

    public void on() {
        mode = Mode.POSITION;
    }

    public double getTargetDegrees() {
        return targetDegrees;
    }

    public void setTargetDegrees(double targetDegrees) {
        this.targetDegrees = legalizeAngle(targetDegrees);
    }

    public void moveLeft() {
        setAngleDegrees(getAngleDegrees() - incrementDegrees);
    }

    public void moveRight() {
        setAngleDegrees(getAngleDegrees() + incrementDegrees);
    }

    public void setStartingAngle(double angle) {
        setAngleDegrees(angle);
    }

    public void usePreviousStartingAngle() {
        setAngleDegrees(angleTransfer);
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case POSITION:
                    controller.updateError(targetDegrees - getAngleDegrees());
                    turretMotor.setPower(controller.run());
                    break;
                case OFF:
                    turretMotor.setPower(0);
                    break;
            }

            angleTransfer = getAngleDegrees();

            context.telemetry.addData("Turret/angle", getAngleDegrees());
            context.telemetry.addData("Turret/target", targetDegrees);
            context.telemetry.addData("Turret/error", targetDegrees - getAngleDegrees());
            context.telemetry.addData("Turret/power", turretMotor.getPower());
            context.telemetry.addData("Turret/mode", mode);
        });
    }

    enum Mode {
        POSITION,
        OFF
    }
}
