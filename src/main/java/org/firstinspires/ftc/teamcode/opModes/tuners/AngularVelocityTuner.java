package org.firstinspires.ftc.teamcode.opModes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.ShootingController;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp(name = "Angular Velocity Tuner", group = "Tuners")
public class AngularVelocityTuner extends RobotOpMode {
    private ShootingController shootingController;

    @Override
    public void init() {
        super.init();
        turret.on();

        drivetrain.usePreviousStartingPose();
        turret.usePreviousStartingAngle();
        shootingController = new ShootingController(context, turret, flywheel, hood);
    }

    @Override
    public void loop() {
        drivetrain.update();

        drivetrain.arcadeDrive(0, 0, 1, Alliance.RED);

        shootingController.prepareForLocation(
                drivetrain.follower.getPose(),
                drivetrain.follower.getVelocity(),
                drivetrain.follower.getAngularVelocity(),
                alliance()
        );

        context.telemetry.addData("gain", (turret.getTargetDegrees() - turret.getAngleDegrees()) / drivetrain.follower.getAngularVelocity());

        super.loop();
    }
}
