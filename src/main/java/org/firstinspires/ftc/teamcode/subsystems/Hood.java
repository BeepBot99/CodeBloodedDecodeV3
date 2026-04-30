package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.Context;

import java.util.function.DoubleUnaryOperator;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public final class Hood {
    public static double shotScalar = 0;
    public static double servoIncrement = 0.01;
    public static double maximumDrop = 0;
    public static double dropDelay = 0;
    private final Servo hoodServo;
    private final Context context;
    private final ElapsedTime shotTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private boolean enabled = true;
    private double target = 0;
    private boolean isShooting = false;

    public Hood(Context context) {
        this.context = context;

        hoodServo = context.servo("hood");
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public void startShot() {
        isShooting = true;
        shotTimer.reset();
    }

    public void endShot() {
        isShooting = false;
    }

    public double getPosition() {
        return target;
    }

    public void setPosition(double position) {
        target = position;
    }

    public void setPosition(DoubleUnaryOperator setter) {
        setPosition(setter.applyAsDouble(getPosition()));
    }

    public void increment() {
        setPosition(position -> position + servoIncrement);
    }

    public void decrement() {
        setPosition(position -> position - servoIncrement);
    }

    public Command periodic() {
        return infinite(() -> {
            setPosition(isShooting ? Math.min(target, Math.max(target - (shotTimer.seconds() - dropDelay) * shotScalar, target - maximumDrop)) : target);
            if (enabled) hoodServo.setPosition(target);
            context.telemetry.addData("Hood/Position", getPosition());
        });
    }
}
