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
                    {0, 0},
                    {0, 0},
                    {0, 0},
            });
    private static final Interpolation2D turretAngleFar = new BilinearInterpolation(
            new double[]{41.8, 0, 0},
            new double[]{4.8, 0},
            new double[][]{
                    {0, 0},
                    {0, 0},
                    {0, 0},
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
                    {0, 0},
                    {0, 0},
                    {0, 0}
            });
    private static final Interpolation2D flywheelVelocityClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {1100, 1100, 1330, 1405},
                    {1220, 1260, 1325, 1405},
                    {1325, 1345, 1395, 1405},
                    {1485, 1470, 1510, 1405}
            }
    );
    private static final Interpolation2D turretAngleClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {7, 32.1, 50, 49.6},
                    {3, 25.1, 37, 49.6},
                    {2.5, 18.2, 33.3, 49.6},
                    {3.7, 13.9, 24.9, 49.6}
            }
    );
    private static final Interpolation2D airTimeClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {0.41, 0.54, 0.38, 0.35},
                    {0.44, 0.38, 0.27, 0.35},
                    {0.45, 0.36, 0.42, 0.35},
                    {0.53, 0.5, 0.39, 0.35}
            }
    );
    private static final Interpolation2D hoodClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {0, 0.25, 0.75, 0.7},
                    {0.55, 0.65, 0.8, 0.7},
                    {0.5, 0.7, 0.68, 0.7},
                    {0.55, 0.7, 0.76, 0.7}
            }
    );
    public static double airTimeMultiplier = 1.6;

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
