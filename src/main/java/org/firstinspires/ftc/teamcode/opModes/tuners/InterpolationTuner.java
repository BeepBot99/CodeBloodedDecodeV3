package org.firstinspires.ftc.teamcode.opModes.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.math.TurretKinematics;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp(name = "Interpolation Tuner", group = "Tuners")
@Config("Interpolation Tuner")
public class InterpolationTuner extends RobotOpMode {
    public static double flywheelVelocity = 0;
    public static double hoodPosition = 0;

    @Override
    public void init() {
        super.init();

        drivetrain.usePreviousStartingPose();
        turret.usePreviousStartingAngle();
    }

    @Override
    public void start() {
        blocker.block();
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

            hood.setPosition(hoodPosition);
            flywheel.setTarget(flywheelVelocity);

            if (gamepad1.rightTriggerWasPressed()) {
                blocker.unblock();
            }

            if (gamepad1.rightTriggerWasReleased()) {
                blocker.block();
            }

            Pose turretPose = TurretKinematics.getTurretPose(drivetrain.follower.getPose());
            context.addPose("Turret/current", turretPose);

            if (gamepad1.rightBumperWasPressed()) intake.off().schedule();
            if (gamepad1.rightBumperWasReleased()) intake.on().schedule();
            if (gamepad1.leftBumperWasPressed()) intake.shortReverse().schedule();

            if (gamepad1.triangleWasPressed()) flywheel.toggle();

            context.telemetry.addData("skibidi turret angle", turret.getAngleDegrees() + Math.toDegrees(drivetrain.follower.getHeading()));
        });
    }
}
