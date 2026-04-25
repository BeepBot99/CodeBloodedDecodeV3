package org.firstinspires.ftc.teamcode.math;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Vector;
import org.firstinspires.ftc.teamcode.util.Alliance;
import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config("Interpolation")
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
                    {72, 71},
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
                    {1030, 1110, 1140, 1270},
                    {1130, 1230, 1280, 1320},
                    {1270, 1330, 1355, 1450},
                    {1420, 1460, 1485, 1580},
            }
    );
    private static final Interpolation2D turretAngleClose = new BilinearInterpolation(
            new double[]{99, 78, 55, 31},
            new double[]{135, 110, 85, 62},
            new double[][]{
                    {3.5, 33, 48, 59},
                    {3.5, 27, 39, 52},
                    {3, 20, 32, 41},
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
    public static double airTimeMultiplier = 1;

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
