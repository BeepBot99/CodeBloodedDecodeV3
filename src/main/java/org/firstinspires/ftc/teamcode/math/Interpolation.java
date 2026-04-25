package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.Alliance;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config("Interpolation")
public final class Interpolation {
    private static final Interpolation2D flywheelVelocityFar = new BilinearInterpolation(
            new double[]{41.8, 0, 0},
            new double[]{4.8, 0},
            new double[][]{
                    {1650, 1680},
                    {1710, 1770},
                    {1760, 1800},
            });
    private static final Interpolation2D turretAngleFar = new BilinearInterpolation(
            new double[]{41.8, 0, 0},
            new double[]{4.8, 0},
            new double[][]{
                    {78, 82},
                    {76, 77},
                    {67, 69},
            });
    private static final Interpolation2D airTimeFar = new BilinearInterpolation(
            new double[]{41.8, 0, 0},
            new double[]{4.8, 0},
            new double[][]{
                    {0, 0},
                    {0, 0},
                    {0, 0}
            });
    private static final Interpolation2D hoodFar = new BilinearInterpolation(
            new double[]{41.8, 0, 0},
            new double[]{4.8, 0},
            new double[][]{
                    {0.8, 0.8},
                    {0.8, 0.8},
                    {0.8, 0.8}
            });
    private static final Interpolation2D flywheelVelocityClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {1030, 1150, 1320, 1320},
                    {1050, 1170, 1350, 1410},
                    {1170, 1250, 1330, 1450},
                    {1300, 1390, 1525, 1570},
            }
    );
    private static final Interpolation2D turretAngleClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {3.5, 45.7, 59.9, 65.1},
                    {5.7, 29.9, 54.8, 61.6},
                    {8.8, 31.8, 35.9, 62},
                    {8.7, 15.4, 24.7, 62.4},
            }
    );
    private static final Interpolation2D airTimeClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {0.32, 0.38, 0.4, 0.42},
                    {0.26, 0.38, 0.43, 0.4},
                    {0.39, 0.33, 0.38, 0.44},
                    {0.3, 0.3, 0.51, 0.46},
            }
    );
    private static final Interpolation2D hoodClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {0.1, 0.5, 0.7, 0.7},
                    {0.3, 0.5, 0.7, 0.8},
                    {0.5, 0.7, 0.7, 0.8},
                    {0.7, 0.7, 0.8, 0.8},
            }
    );
    public static double airTimeMultiplier = 1.8;

    private static double interpolate(Interpolation2D closeInterpolation, Interpolation2D farInterpolation, Vector pose, Alliance alliance) {
        if (alliance == Alliance.BLUE) {
            pose = new Vector();
            pose.setOrthogonalComponents(141.5 - pose.getXComponent(), pose.getYComponent());
        }
        if (pose.getYComponent() > 48) {
            return closeInterpolation.interpolate(pose.getXComponent(), pose.getYComponent());
        } else {
            return farInterpolation.interpolate(pose.getXComponent(), pose.getYComponent());
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
        return interpolate(airTimeClose, airTimeFar, pose, alliance) * airTimeMultiplier;
    }

    public static double getHood(Vector pose, Alliance alliance) {
        return interpolate(hoodClose, hoodFar, pose, alliance);
    }
}
