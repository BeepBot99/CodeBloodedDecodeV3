package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config("Settings")
public final class Context {
    public static boolean useAdvantageScopeNotation = false;
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;

    public Context(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = new MultipleTelemetry(
                opMode.telemetry,
                FtcDashboard.getInstance().getTelemetry(),
                PanelsTelemetry.INSTANCE.getFtcTelemetry()
        );
    }

    public void addPose(String name, Pose pose) {
        if (useAdvantageScopeNotation) {
            telemetry.addData(name + " x", 144 - pose.getX());
            telemetry.addData(name + " y", 144 - pose.getY());
            telemetry.addData(name + " heading (deg)", 180 + Math.toDegrees(pose.getHeading()));
        } else {
            telemetry.addData(name + " x", pose.getX());
            telemetry.addData(name + " y", pose.getY());
            telemetry.addData(name + " heading (deg)", Math.toDegrees(pose.getHeading()));
        }
    }
}
