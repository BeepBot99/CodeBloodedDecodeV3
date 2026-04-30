package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.Alliance;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

public final class Interpolation {
    private static final Interpolation2D flywheelVelocityFar = new BilinearInterpolation(
            new double[]{41.8, 71, 101.2},
            new double[]{5, 29.3},
            new double[][]{
                    {1750, 1620},
                    {1660, 1570},
                    {1575, 1440},
            });
    private static final Interpolation2D turretAngleFar = new BilinearInterpolation(
            new double[]{41.8, 71, 101.2},
            new double[]{5, 29.3},
            new double[][]{
                    {54, 49},
                    {60, 61},
                    {74, 71},
            });
    private static final Interpolation2D airTimeFar = new BilinearInterpolation(
            new double[]{41.8, 71, 101.2},
            new double[]{5, 29.3},
            new double[][]{
                    {1.00, 0.82},
                    {0.92, 0.87},
                    {0.93, 0.84}
            });
    private static final Interpolation2D hoodFar = new BilinearInterpolation(
            new double[]{41.8, 71, 101.2},
            new double[]{5, 29.3},
            new double[][]{
                    {0.75, 0.76},
                    {0.75, 0.75},
                    {0.75, 0.7}
            });
    private static final Interpolation2D flywheelVelocityClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {1030, 1110, 1125, 1270},
                    {1130, 1230, 1280, 1320},
                    {1270, 1330, 1355, 1450},
                    {1420, 1460, 1485, 1580},
            }
    );
    private static final Interpolation2D turretAngleClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {3.5, 33, 49.5, 59},
                    {3.5, 27, 39, 52},
                    {3, 21, 32, 41},
                    {3, 12, 29.5, 33},
            }
    );
    private static final Interpolation2D airTimeClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {0.63, 0.56, 0.58, 0.68},
                    {0.63, 0.53, 0.62, 0.73},
                    {0.72, 0.68, 0.75, 0.72},
                    {0.65, 0.84, 0.84, 0.85},
            }
    );
    private static final Interpolation2D hoodClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {0.15, 0.4, 0.6, 0.65},
                    {0.4, 0.7, 0.7, 0.65},
                    {0.6, 0.7, 0.65, 0.7},
                    {0.85, 0.6, 0.725, 0.775},
            }
    );

    private static double interpolate(Interpolation2D closeInterpolation, Interpolation2D farInterpolation, Vector pose, Alliance alliance) {
        final Vector newPose;
        if (alliance == Alliance.BLUE) {
            newPose = new Vector();
            newPose.setOrthogonalComponents(141.5 - pose.getXComponent(), pose.getYComponent());
        } else {
            newPose = pose;
        }
        if (newPose.getYComponent() > 48) {
            return closeInterpolation.interpolate(newPose.getXComponent(), newPose.getYComponent());
        } else {
            return farInterpolation.interpolate(newPose.getXComponent(), newPose.getYComponent());
        }
    }

    public static double getFlywheelVelocity(Vector pose, Alliance alliance) {
        return interpolate(flywheelVelocityClose, flywheelVelocityFar, pose, alliance);
    }

    public static double getTurretAngle(Vector pose, Alliance alliance) {
        double angle =  interpolate(turretAngleClose, turretAngleFar, pose, alliance);
        if (alliance == Alliance.BLUE) angle = 180 - angle;
        return angle;
    }

    public static double getAirTime(Vector pose, Alliance alliance) {
        return interpolate(airTimeClose, airTimeFar, pose, alliance);
    }

    public static double getHood(Vector pose, Alliance alliance) {
        return interpolate(hoodClose, hoodFar, pose, alliance);
    }
}
