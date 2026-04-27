package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.*;

@Config
public final class Intake {
    public static boolean logCurrent = false;
    public static double onPower = -1;
    public static double offPower = 0;
    public static double reversePower = 1;
    public static double shortReverseTimeMs = 150;
    private final DcMotorEx intakeMotor;
    private final Context context;
    private Mode mode = Mode.OFF;

    public Intake(Context context) {
        this.context = context;
        intakeMotor = context.motor("intake");
    }

    public Command on() {
        return instant(() -> mode = Mode.ON).requiring(intakeMotor);
    }

    public Command off() {
        return instant(() -> mode = Mode.OFF).requiring(intakeMotor);
    }

    public Command reverse() {
        return instant(() -> mode = Mode.REVERSE).requiring(intakeMotor);
    }

    public Command shortReverse() {
        return reverse().then(waitMs(shortReverseTimeMs)).then(on()).setConflictBehavior(ConflictBehavior.OVERRIDE);
    }

    public Command toggle() {
        return conditional(() -> mode == Mode.OFF, on(), off());
    }

    public Command periodic() {
        return infinite(() -> {
            switch (mode) {
                case ON:
                    intakeMotor.setPower(onPower);
                    break;
                case OFF:
                    intakeMotor.setPower(offPower);
                    break;
                case REVERSE:
                    intakeMotor.setPower(reversePower);
                    break;
            }

            if (logCurrent) context.telemetry.addData("Intake/current", intakeMotor.getCurrent(CurrentUnit.AMPS));
            context.telemetry.addData("Intake/velocity", intakeMotor.getVelocity());
            context.telemetry.addData("Intake/power", intakeMotor.getPower());
        });
    }

    public double getCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }

    enum Mode {
        ON,
        OFF,
        REVERSE
    }
}
