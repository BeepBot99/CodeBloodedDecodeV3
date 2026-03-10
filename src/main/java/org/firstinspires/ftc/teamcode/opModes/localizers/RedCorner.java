package org.firstinspires.ftc.teamcode.opModes.localizers;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;

@Autonomous(name = "Red Corner", group = "Localizers")
public class RedCorner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret.localize(0);
        Drivetrain.localize(new Pose(7.5, 8.1, Math.PI / 2));
        Alliance.current = Alliance.RED;
    }
}
