package org.firstinspires.ftc.teamcode.control;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.math.Interpolation;
import org.firstinspires.ftc.teamcode.math.SOTM;
import org.firstinspires.ftc.teamcode.math.TurretKinematics;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Context;

public class ShootingController {
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

        double flywheelVelocity = Interpolation.getFlywheelVelocity(virtualTurretPose);
        double turretAngle = Interpolation.getTurretAngle(virtualTurretPose);

        flywheel.setTarget(flywheelVelocity);
        turret.setTargetDegrees(turretAngle);

        context.telemetry.addData("Turret/current x", turretPose.getX());
        context.telemetry.addData("Turret/current y", turretPose.getY());
        context.telemetry.addData("Turret/current heading (deg)", Math.toDegrees(turretPose.getHeading()));

        context.telemetry.addData("Turret/velocity x", turretVelocity.getXComponent());
        context.telemetry.addData("Turret/velocity y", turretVelocity.getYComponent());

        context.telemetry.addData("SOTM/virtual turret x", virtualTurretPose.getXComponent());
        context.telemetry.addData("SOTM/virtual turret y", virtualTurretPose.getYComponent());
        context.telemetry.addData("SOTM/virtual turret heading (deg)", Math.toDegrees(turretPose.getHeading()));
    }
}
