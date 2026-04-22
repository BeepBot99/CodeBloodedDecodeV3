package org.firstinspires.ftc.teamcode.control;

import android.util.Size;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.util.Context;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public final class AprilTagLocalizer {
    private static final Position cameraPosition = new Position(
            DistanceUnit.INCH,
            2.204,
            5.96,
            8.82,
            0
    );

    private static final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -80,
            180,
            0
    );

    private final Context context;
    private final AprilTagProcessor processor;
    private final FusionLocalizer fusion;

    public Localizer getLocalizer() {
        return fusion;
    }

    public AprilTagLocalizer(Context context, Localizer localizer) {
        this.context = context;

        fusion = new FusionLocalizer(
                localizer,
                new Pose(0.25, 0.25, Math.toRadians(2)),
                new Pose(1.5 / 60, 1.5 / 60, Math.toRadians(0.5) / 60),
                new Pose(2.1561, 2.6065, 0.0248),
                100
        );

        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(544.2876017217492, 543.8059217350639, 332.20336755183894, 248.65289514406953)
                .build();

        new VisionPortal.Builder()
                .setCamera(context.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(processor)
                .build();
    }

    public void update() {
        List<AprilTagDetection> detections = processor.getDetections();

        context.telemetry.addData("AprilTags/Detections", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null || detection.metadata.name.contains("Obelisk")) continue;

            Pose pose = new Pose(
                    detection.robotPose.getPosition().y + 72,
                    -detection.robotPose.getPosition().x + 72,
                    detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
            );

            context.addPose("AprilTags/" + detection.metadata.name, pose);

            fusion.addMeasurement(pose, System.nanoTime() - 50_000_000);
        }
    }
}
