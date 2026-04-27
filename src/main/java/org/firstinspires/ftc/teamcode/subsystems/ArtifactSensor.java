package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.infinite;

@Config("Artifact Sensor")
public final class ArtifactSensor {
    public static double holdTimeMs = 400;
    private final DigitalChannel threeBallSensor;
    private final Context context;
    private boolean triggered;
    private long triggeredAt = Long.MAX_VALUE;

    public ArtifactSensor(Context context) {
        this.context = context;
        threeBallSensor = context.hardwareMap.get(DigitalChannel.class, "threeBallSensor");
        threeBallSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean hasThree() {
        return triggered && System.nanoTime() - triggeredAt >= holdTimeMs * 1e6;
    }

    public void update() {
        boolean wasTriggered = triggered;
        triggered = threeBallSensor.getState();
        if (triggered && !wasTriggered) triggeredAt = System.nanoTime();
    }

    public Command periodic() {
        return infinite(() -> {
            context.telemetry.addData("Sensors/ThreeBalls/state", hasThree());
        });
    }
}
