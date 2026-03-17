package org.firstinspires.ftc.teamcode.opModes.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp(name = "Turret Tuner", group = "Tuners")
@Config("Turret Tuner")
public class TurretTuner extends RobotOpMode {
    public static boolean on = false;
    public static double target = 0;
    public static boolean waveEnabled = false;
    public static double wavePeriod = 1;
    public static double waveHigh = 90;
    public static double waveLow = 0;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        super.init();
        turret.setStartingAngle(0);
    }

    @Override
    public void loop() {
        if (on) {
            turret.on();
            if (waveEnabled) {
                if (timer.seconds() > wavePeriod / 2) {
                    timer.reset();
                    target = turret.getTargetDegrees() == waveLow ? waveHigh : waveLow;
                }
            }
            turret.setTargetDegrees(target);
        } else {
            turret.off();
        }
        super.loop();
    }
}
