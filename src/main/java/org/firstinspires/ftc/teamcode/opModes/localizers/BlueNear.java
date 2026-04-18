package org.firstinspires.ftc.teamcode.opModes.localizers;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@Autonomous(name = "Blue Near", group = "Localizers")
public class BlueNear extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret.localize(0);
        Drivetrain.localize(new Pose(118, 123, Math.toRadians(36.5)).mirror());
        RobotOpMode.setAlliance(Alliance.BLUE);
    }
}
