package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Context;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Config
public final class Drivetrain {
    public static PIDFCoefficients headingCoefficients = new PIDFCoefficients(1.75, 0, 0.09, 0);
    public static double gateOpenHeadingDegrees = 36.5;
    private static Pose poseTransfer = new Pose();
    public final DcMotorEx frontLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backLeft;
    public final DcMotorEx backRight;
    public final Follower follower;
    private final Context context;
    private final PIDFController headingController = new PIDFController(headingCoefficients);
    private boolean lockHeading = false;
    private double headingTargetRadians = 0;

    public Drivetrain(Context context) {
        this.context = context;
        follower = Constants.createFollower(context.hardwareMap);
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

    public void unlockHeading() {
        lockHeading = false;
    }

    public void arcadeDrive(double forward, double strafe, double turn, Alliance alliance) {
        double headingRadians = follower.getHeading();

        forward = signedSquare(forward);
        strafe = signedSquare(strafe);

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
    }

    public Command periodic() {
        return infinite(() -> {
            poseTransfer = follower.getPose();

            context.addPose("Drivetrain/current", follower.getPose());
            context.telemetry.addData("Drivetrain/heading locked", lockHeading);
            context.telemetry.addData("Drivetrain/heading target", headingTargetRadians);
        });
    }
}
