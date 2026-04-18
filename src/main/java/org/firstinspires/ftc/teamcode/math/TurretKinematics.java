package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

public final class TurretKinematics {
    public static double turretOffsetInches = -3;

    public static Pose getTurretPose(Pose robotPose) {
        return new Pose(
                robotPose.getX() + turretOffsetInches * Math.cos(robotPose.getHeading()),
                robotPose.getY() + turretOffsetInches * Math.sin(robotPose.getHeading()),
                robotPose.getHeading()
        );
    }

    public static Vector getTurretVelocity(Vector robotVelocity, double angularVelocity, double robotHeadingRadians) {
        Vector result = new Vector();
        result.setOrthogonalComponents(
                robotVelocity.getXComponent() - angularVelocity * turretOffsetInches * Math.sin(robotHeadingRadians),
                robotVelocity.getYComponent() + angularVelocity * turretOffsetInches * Math.cos(robotHeadingRadians)
        );
        return result;
    }
}

