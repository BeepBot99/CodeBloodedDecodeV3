package org.firstinspires.ftc.teamcode.math;

import com.pedropathing.math.Vector;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public final class Interpolation {
    private static final Interpolation2D flywheelVelocityFar = new BilinearInterpolation(
            new double[]{43, 71, 100},
            new double[]{6, 27},
            new double[][]{
                    {1675, 1640},
                    {1675, 1520},
                    {1620, 1520}
            });

    private static final Interpolation2D turretAngleFar = new BilinearInterpolation(
            new double[]{43, 71, 100},
            new double[]{6, 27},
            new double[][]{
                    {55.2, 54},
                    {61, 58},
                    {79, 75}
            });

    private static final Interpolation2D airTimeFar = new BilinearInterpolation(
            new double[]{43, 71, 100},
            new double[]{6, 27},
            new double[][]{
                    {0, 0},
                    {0, 0},
                    {0, 0}
            });

    private static final Interpolation2D flywheelVelocityClose = new BilinearInterpolation(
            new double[]{38, 61, 85},
            new double[]{135.5, 111, 88, 63},
            new double[][]{
                    {1400, 1430, 1470, 1520},
                    {1375, 1380, 1390, 1415},
                    {1338, 1325, 1330, 1350},
            }
    );

    private static final Interpolation2D turretAngleClose = new BilinearInterpolation(
            new double[]{38, 61, 85},
            new double[]{135.5, 111, 88, 63},
            new double[][]{
                    {1.3, 2.8, 27.5, 37.5},
                    {3, 16.5, 32, 45},
                    {5.5, 28, 41, 56},
            }
    );

    private static final Interpolation2D airTimeClose = new BilinearInterpolation(
            new double[]{38, 61, 85},
            new double[]{135.5, 111, 88, 63},
            new double[][]{
                    {0, 0, 0, 0},
                    {0, 0, 0, 0},
                    {0, 0, 0, 0},
            }
    );

    private static double interpolate(Interpolation2D closeInterpolation, Interpolation2D farInterpolation, Vector pose) {
        if (pose.getYComponent() > 48) {
            return closeInterpolation.interpolate(pose.getXComponent(), pose.getYComponent());
        } else {
            return farInterpolation.interpolate(pose.getXComponent(), pose.getYComponent());
        }
    }

    public static double getFlywheelVelocity(Vector pose) {
        return interpolate(flywheelVelocityClose, flywheelVelocityFar, pose);
    }

    public static double getTurretAngle(Vector pose) {
        return interpolate(turretAngleClose, turretAngleFar, pose);
    }

    public static double getAirTime(Vector pose) {
        return interpolate(airTimeClose, airTimeFar, pose);
    }
}
