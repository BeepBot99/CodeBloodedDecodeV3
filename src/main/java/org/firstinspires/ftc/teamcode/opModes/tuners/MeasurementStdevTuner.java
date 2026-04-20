package org.firstinspires.ftc.teamcode.opModes.tuners;

import android.util.Size;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;
import org.firstinspires.ftc.teamcode.util.WelfordVariance;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Measurement STDEV Tuner", group = "Tuners")
@Config("Measurement STDEV Tuner")
public class MeasurementStdevTuner extends RobotOpMode {

    private static final Position cameraPosition = new Position(
            DistanceUnit.INCH,
            2.2782,
            8.139,
            8.3053,
            0
    );
    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -80,
            180,
            0
    );
    public static boolean on = false;
    public static double actualX = 0;
    public static double actualY = 0;
    public static double actualHeading = 0;
    private final WelfordVariance varianceX = new WelfordVariance();
    private final WelfordVariance varianceY = new WelfordVariance();
    private final WelfordVariance varianceHeading = new WelfordVariance();
    private AprilTagProcessor processor;

    @Override
    public void init() {
        super.init();

        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(544.2876017217492, 543.8059217350639, 332.20336755183894, 248.65289514406953)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();
    }

    @Override
    public void loop() {
        wrapLoop(() -> {
            drivetrain.arcadeDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    alliance()
            );

            if (on) {
                List<AprilTagDetection> detections = processor.getDetections();
                context.telemetry.addData("AprilTag/Detections", detections.size());

                for (AprilTagDetection detection : detections) {
                    if (detection.metadata == null || detection.metadata.name.contains("Obelisk")) continue;

                    context.telemetry.addData(detection.metadata.name, detection.robotPose);

                    Pose pose = new Pose(
                            detection.robotPose.getPosition().y + 72,
                            -detection.robotPose.getPosition().x + 72,
                            detection.robotPose.getOrientation().getYaw()
                    );

                    context.addPose(detection.metadata.name, pose);

                    varianceX.update(actualX - pose.getX());
                    varianceY.update(actualY - pose.getY());
                    varianceHeading.update(actualHeading - pose.getHeading());
                }
            }

            context.telemetry.addData("variance x", varianceX.variance());
            context.telemetry.addData("variance y", varianceY.variance());
            context.telemetry.addData("variance heading", varianceHeading.variance());

            context.telemetry.addData("stdev x", varianceX.stdDev());
            context.telemetry.addData("stdev y", varianceY.stdDev());
            context.telemetry.addData("stdev heading", varianceHeading.stdDev());

            context.telemetry.addData("mean x", varianceX.mean());
            context.telemetry.addData("mean y", varianceY.mean());
            context.telemetry.addData("mean heading", varianceHeading.mean());
        });
    }
}
