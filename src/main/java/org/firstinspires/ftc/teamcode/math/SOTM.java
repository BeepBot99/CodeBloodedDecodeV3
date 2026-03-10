package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Vector;

public final class SOTM {
    private static final int ITERATIONS = 5;

    public static Vector calculateVirtualRobot(Vector robotPose, Vector robotVelocity) {
        Vector virtualPose = robotPose;
        for (int i = 0; i < ITERATIONS; i++) {
            double airTime = Interpolation.getAirTime(virtualPose);
            virtualPose = robotPose.minus(robotVelocity.times(airTime));
        }
        return virtualPose;
    }
}
