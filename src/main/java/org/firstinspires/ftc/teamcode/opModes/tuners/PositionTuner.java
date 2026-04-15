package org.firstinspires.ftc.teamcode.opModes.tuners;

import android.util.Size;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.FusionLocalizer;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Position Tuner", group = "Tuners")
public class PositionTuner extends RobotOpMode {

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
            0,
            0
    );

    private AprilTagProcessor processor;
    private VisionPortal visionPortal;
    private Follower follower;
    private FusionLocalizer fusion;

    @Override
    public void init() {

        fusion = new FusionLocalizer(
                new PinpointLocalizer(hardwareMap, Constants.localizerConstants),
                new Pose(0.5, 0.5, Math.toRadians(2)),
                new Pose(0.05, 0.05, Math.toDegrees(0.5)),
                new Pose(2, 2, Math.toDegrees(3)),
                100
        );

        follower = new FollowerBuilder(Constants.followerConstants, hardwareMap)
                .mecanumDrivetrain(Constants.driveConstants)
                .setLocalizer(fusion)
                .pathConstraints(Constants.pathConstraints)
                .build();

        super.init();

        drivetrain.usePreviousStartingPose();

        processor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(544.2876017217492, 543.8059217350639, 332.20336755183894, 248.65289514406953)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .build();
    }

    @Override
    public void loop() {
        drivetrain.update();

        drivetrain.arcadeDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                alliance()
        );

        List<AprilTagDetection> detections = processor.getDetections();
        context.telemetry.addData("AprilTag/Detections", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.metadata == null || detection.metadata.name.contains("Obelisk")) continue;

            Pose2D sdkPose = new Pose2D(
                    DistanceUnit.INCH,
                    detection.robotPose.getPosition().x,
                    detection.robotPose.getPosition().y,
                    AngleUnit.DEGREES,
                    detection.robotPose.getOrientation().getYaw()
            );

            Pose ftcPose = PoseConverter.pose2DToPose(sdkPose, InvertedFTCCoordinates.INSTANCE);

            Pose pose = ftcPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);

            context.addPose(detection.metadata.name, pose);

            fusion.addMeasurement(pose, System.nanoTime());
        }

        context.addPose("Fusion", follower.getPose());

        super.loop();
    }
}
