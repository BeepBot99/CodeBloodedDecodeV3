package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Context;

import java.util.function.DoubleUnaryOperator;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public final class Hood {
    public static double servoIncrement = 0.01;

    private final Servo hoodServo;
    private final Context context;

    public Hood(Context context) {
        this.context = context;

        hoodServo = context.servo("hood");
    }

    public double getPosition() {
        return hoodServo.getPosition();
    }

    public void setPosition(double position) {
        hoodServo.setPosition(position);
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
            context.telemetry.addData("Hood/Position", getPosition());
        });
    }
}
