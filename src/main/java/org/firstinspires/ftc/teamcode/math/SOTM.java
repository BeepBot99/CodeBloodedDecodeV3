package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.Alliance;

public final class SOTM {
    private static final int ITERATIONS = 10;

    public static Vector calculateVirtualRobot(Vector robotPose, Vector robotVelocity, Alliance alliance) {
        if (robotVelocity.getMagnitude() < 1) return robotPose;
        Vector virtualPose = robotPose;
        for (int i = 0; i < ITERATIONS; i++) {
            double airTime = Interpolation.getAirTime(virtualPose, alliance);
            virtualPose = robotPose.plus(robotVelocity.times(airTime));
        }
        return virtualPose;
    }
}
