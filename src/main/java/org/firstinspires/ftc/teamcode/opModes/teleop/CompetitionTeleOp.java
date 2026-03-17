package org.firstinspires.ftc.teamcode.opModes.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.ShootingController;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp(name = "TeleOp", group = "Competition")
@Config("TeleOp")
public class CompetitionTeleOp extends RobotOpMode {
    private ShootingController shootingController;

    @Override
    public void init() {
        super.init();

        drivetrain.usePreviousStartingPose();
        turret.usePreviousStartingAngle();
        shootingController = new ShootingController(context, turret, flywheel);
    }

    @Override
    public void start() {
        blocker.block();
        turret.on();
    }

    @Override
    public void loop() {
        drivetrain.update();

        if (Math.abs(gamepad1.right_stick_x) >= 0.1) drivetrain.unlockHeading();

        drivetrain.arcadeDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                alliance()
        );

        shootingController.prepareForLocation(
                drivetrain.follower.getPose(),
                drivetrain.follower.getVelocity(),
                drivetrain.follower.getAngularVelocity()
        );

        if (drivetrain.follower.getPose().getY() < 48 && gamepad1.right_trigger >= 0.1) {
            intake.slowDown();
        } else {
            intake.speedUp();
        }

        if (gamepad1.rightTriggerWasPressed()) {
            blocker.unblock();
        }

        if (gamepad1.rightTriggerWasReleased()) {
            blocker.block();
        }

        if (gamepad1.rightBumperWasPressed()) intake.off().schedule();
        if (gamepad1.rightBumperWasReleased()) intake.on().schedule();
        if (gamepad1.leftBumperWasPressed()) intake.shortReverse().schedule();

        if (gamepad1.triangleWasPressed()) flywheel.toggle();

        if (gamepad1.squareWasPressed()) drivetrain.gateHeading(alliance());

        if (gamepad2.leftTriggerWasPressed()) blocker.assembly();

        if (gamepad2.leftBumperWasPressed()) setAlliance(Alliance.RED);
        if (gamepad2.rightBumperWasPressed()) setAlliance(Alliance.BLUE);

        if (gamepad2.crossWasPressed()) drivetrain.follower.setPose(
                alliance() == Alliance.RED ? new Pose(8.1, 7.5, 0) : new Pose(141.5 - 8.1, 7.5, Math.PI)
        );

        if (gamepad2.circleWasPressed()) drivetrain.follower.setPose(drivetrain.follower.getPose().withHeading(
                alliance() == Alliance.RED ? 0 : Math.PI
        ));

        if (gamepad2.dpadLeftWasPressed()) turret.moveLeft();
        if (gamepad2.dpadRightWasPressed()) turret.moveRight();

        super.loop();
    }
}
