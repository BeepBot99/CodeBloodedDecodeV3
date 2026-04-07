package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.math.Interpolation;
import org.firstinspires.ftc.teamcode.math.SOTM;
import org.firstinspires.ftc.teamcode.math.TurretKinematics;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Context;

@Config("Shooting Controller")
public final class ShootingController {
    public static double turningGain = 3.45;
    private final Context context;
    private final Turret turret;
    private final Flywheel flywheel;

    public ShootingController(Context context, Turret turret, Flywheel flywheel) {
        this.context = context;
        this.turret = turret;
        this.flywheel = flywheel;
    }

    public void prepareForLocation(Pose robotPose, Vector robotVelocity, double angularVelocity) {
        Pose turretPose = TurretKinematics.getTurretPose(robotPose);

        Vector turretVelocity = TurretKinematics.getTurretVelocity(robotVelocity, angularVelocity, robotPose.getHeading());

        Vector virtualTurretPose = SOTM.calculateVirtualRobot(turretPose.getAsVector(), turretVelocity);
//        Vector virtualTurretPose = turretPose.getAsVector();

        double flywheelVelocity = Interpolation.getFlywheelVelocity(virtualTurretPose);
        double turretAngle = Interpolation.getTurretAngle(virtualTurretPose);

        flywheel.setTarget(flywheelVelocity);
        turret.setTargetDegrees(turretAngle - Math.toDegrees(robotPose.getHeading()) - turningGain * angularVelocity);

        context.addPose("Turret/current", turretPose);

        context.telemetry.addData("Turret/velocity x", turretVelocity.getXComponent());
        context.telemetry.addData("Turret/velocity y", turretVelocity.getYComponent());

        context.addPose("SOTM/virtual turret", new Pose(virtualTurretPose.getXComponent(), virtualTurretPose.getYComponent(), robotPose.getHeading()));
    }
}
