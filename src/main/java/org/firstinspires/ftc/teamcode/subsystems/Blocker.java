package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config
public final class Blocker {
    public static double blockPosition = 0.415;
    public static double unblockPosition = 0.24;

    private final Servo blockerServo;
    private final Context context;

    public Blocker(Context context) {
        this.context = context;
        blockerServo = context.hardwareMap.get(Servo.class, "blocker");
    }

    public void block() {
        blockerServo.setPosition(blockPosition);
    }

    public void unblock() {
        blockerServo.setPosition(unblockPosition);
    }

    public Command periodic() {
        return infinite(() -> {
            context.telemetry.addData("Blocker/position", blockerServo.getPosition());
        });
    }
}
