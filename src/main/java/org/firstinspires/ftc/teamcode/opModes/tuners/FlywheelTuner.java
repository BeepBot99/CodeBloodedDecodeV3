package org.firstinspires.ftc.teamcode.opModes.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp(name = "Flywheel Tuner", group = "Tuners")
@Config("Flywheel Tuner")
public class FlywheelTuner extends RobotOpMode {
    public static boolean on = false;
    public static double target = 0;

    @Override
    public void loop() {
        if (on) {
            flywheel.on();
            flywheel.setTarget(target);
        } else {
            flywheel.off();
        }
        super.loop();
    }
}
