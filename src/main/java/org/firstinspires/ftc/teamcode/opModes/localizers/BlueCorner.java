package org.firstinspires.ftc.teamcode.opModes.localizers;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@Autonomous(name = "Blue Corner", group = "Localizers")
public class BlueCorner extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret.localize(0);
        Drivetrain.localize(new Pose(6.5, 9, Math.PI / 2).mirror());
        RobotOpMode.setAlliance(Alliance.BLUE);
    }
}

