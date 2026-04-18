package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.math.Interpolation;
import org.firstinspires.ftc.teamcode.math.SOTM;
import org.firstinspires.ftc.teamcode.math.TurretKinematics;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Context;

@Config("Shooting Controller")
public final class ShootingController {
    public static double turningGain = 3.8;
    private final Context context;
    private final Turret turret;
    private final Flywheel flywheel;
    private final Hood hood;

    public ShootingController(Context context, Turret turret, Flywheel flywheel, Hood hood) {
        this.context = context;
        this.turret = turret;
        this.flywheel = flywheel;
        this.hood = hood;
    }

    public void prepareForLocation(Pose robotPose, Vector robotVelocity, double angularVelocity, Alliance alliance) {
        Pose turretPose = TurretKinematics.getTurretPose(robotPose);

        Vector turretVelocity = TurretKinematics.getTurretVelocity(robotVelocity, angularVelocity, robotPose.getHeading());

        Vector virtualTurretPose = SOTM.calculateVirtualRobot(turretPose.getAsVector(), turretVelocity, alliance);
//        Vector virtualTurretPose = turretPose.getAsVector();

        double flywheelVelocity = Interpolation.getFlywheelVelocity(virtualTurretPose, alliance);
        double turretAngle = Interpolation.getTurretAngle(virtualTurretPose, alliance);

        flywheel.setTarget(flywheelVelocity);
        turret.setTargetDegrees(turretAngle - Math.toDegrees(robotPose.getHeading()) - turningGain * angularVelocity);

        double hoodPosition = Interpolation.getHood(virtualTurretPose, alliance);
        hood.setPosition(hoodPosition);

        context.addPose("Turret/current", turretPose);

        context.telemetry.addData("Turret/velocity x", turretVelocity.getXComponent());
        context.telemetry.addData("Turret/velocity y", turretVelocity.getYComponent());

        context.addPose("SOTM/virtual turret", new Pose(virtualTurretPose.getXComponent(), virtualTurretPose.getYComponent(), robotPose.getHeading()));
    }
}
