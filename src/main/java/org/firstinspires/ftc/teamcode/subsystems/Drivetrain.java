package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.control.PredictiveBrakingController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.control.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Config
public final class Drivetrain implements AutoCloseable {
    public static PIDFCoefficients headingCoefficients = new PIDFCoefficients(1.75, 0, 0.09, 0);
    public static double gateOpenHeadingDegrees = 25;
    private static Pose poseTransfer = new Pose();
    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final Follower follower;
    private final Context context;
    private final PIDFController headingController = new PIDFController(headingCoefficients);
    private final AprilTagLocalizer aprilTags;
    private final PredictiveBrakingController positionXController = new PredictiveBrakingController(Constants.followerConstants.predictiveBrakingCoefficients);
    private final PredictiveBrakingController positionYController = new PredictiveBrakingController(Constants.followerConstants.predictiveBrakingCoefficients);
    private boolean lockHeading = false;
    private double headingTargetRadians = 0;
    private boolean lockPosition = false;
    private double positionTargetX = 0;
    private double positionTargetY = 0;

    public Drivetrain(Context context) {
        this.context = context;

        aprilTags = new AprilTagLocalizer(
                context,
                new PinpointLocalizer(context.hardwareMap, Constants.localizerConstants)
        );

        follower = new FollowerBuilder(Constants.followerConstants, context.hardwareMap)
                .mecanumDrivetrain(Constants.driveConstants)
                .pathConstraints(Constants.pathConstraints)
                .setLocalizer(aprilTags.getLocalizer())
                .build();

        frontLeft = context.motor("frontLeft");
        frontRight = context.motor("frontRight");
        backLeft = context.motor("backLeft");
        backRight = context.motor("backRight");

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private static double signedSquare(double raw) {
        return Math.signum(raw) * Math.pow(raw, 2);
    }

    public static void localize(Pose pose) {
        poseTransfer = pose;
    }

    public void gateHeading(Alliance alliance) {
        lockHeading = true;
        if (alliance == Alliance.RED) {
            headingTargetRadians = Math.toRadians(gateOpenHeadingDegrees);
        } else {
            headingTargetRadians = Math.PI - Math.toRadians(gateOpenHeadingDegrees);
        }
    }

    public void lockCurrentHeading() {
        if (lockHeading) return;
        lockHeading = true;
        headingTargetRadians = follower.getHeading();
    }

    public void unlockHeading() {
        lockHeading = false;
    }

    public void lockCurrentPosition() {
        if (lockPosition) return;
        lockPosition = true;
        positionTargetX = follower.getPose().getX();
        positionTargetY = follower.getPose().getY();
    }

    public void unlockPosition() {
        lockPosition = false;
    }

    public void arcadeDrive(double forward, double strafe, double turn, Alliance alliance) {
        double headingRadians = follower.getHeading();

        if (lockPosition) {
            forward = positionXController.computeOutput(positionTargetX - follower.getPose().getX(), follower.getVelocity().getXComponent());
            strafe = positionYController.computeOutput(positionTargetY - follower.getPose().getY(), follower.getVelocity().getYComponent());
        } else {
            forward = signedSquare(forward);
            strafe = signedSquare(strafe);
        }

        if (lockHeading) {
            headingController.updateError(AngleUnit.normalizeRadians(headingTargetRadians - headingRadians));
            turn = -headingController.run();
        } else {
            turn = signedSquare(turn);
        }

        if (alliance == Alliance.BLUE) headingRadians += Math.PI;

        double x = strafe * Math.cos(headingRadians) + forward * Math.sin(headingRadians);
        double y = strafe * -Math.sin(headingRadians) + forward * Math.cos(headingRadians);

        y *= 1.1;

        if (lockPosition) {
            Vector robotVelocity = follower.getVelocity();
            robotVelocity.rotateVector(-follower.getHeading());
            if (Math.signum(x) != Math.signum(robotVelocity.getXComponent())) x = Math.signum(x) * 0.2;
            if (Math.signum(y) != Math.signum(robotVelocity.getYComponent())) y = Math.signum(y) * 0.2;
        }

        double denominator = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(turn), 1);

        frontLeft.setPower((y + x + turn) / denominator);
        frontRight.setPower((y - x - turn) / denominator);
        backLeft.setPower((y - x + turn) / denominator);
        backRight.setPower((y + x - turn) / denominator);
    }

    public void usePreviousStartingPose() {
        follower.setStartingPose(poseTransfer);
    }

    public Command followPath(PathChain path) {
        return follow(follower, path);
    }

    public void update() {
        follower.update();
        aprilTags.update();
    }

    public Command periodic() {
        return infinite(() -> {
            poseTransfer = follower.getPose();

            context.addPose("Drivetrain/current", follower.getPose());
            context.telemetry.addData("Drivetrain/heading locked", lockHeading);
            context.telemetry.addData("Drivetrain/heading target", headingTargetRadians);
        });
    }

    @Override
    public void close() {
        aprilTags.close();
    }
}
