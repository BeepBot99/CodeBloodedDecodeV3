package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.control.ShootingController;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;

@Autonomous
public class RedNearGate extends RobotOpMode {

    private Paths paths;
    private ShootingController shootingController;

    @Override
    public void init() {
        super.init();

        drivetrain.follower.setStartingPose(new Pose(116, 127.5, Math.toRadians(-142.5)));
        turret.setStartingAngle(0);
        blocker.block();
        turret.on();

        paths = new Paths(drivetrain.follower);
        shootingController = new ShootingController(
                context,
                turret,
                flywheel,
                hood
        );
    }

    private Command shoot() {
        return sequential(
                instant(blocker::unblock),
                waitMs(1200),
                instant(blocker::block)
        );
    }

    @Override
    public void start() {
        schedule(
                sequential(
                        instant(flywheel::on),
                        intake.on(),
                        waitUntil(() -> flywheel.getVelocity() >= flywheel.getTarget() * 0.9),
//                        waitUntil(flywheel::atTarget),
                        parallel(
                                sequential(
                                        instant(() -> drivetrain.follower.setMaxPower(0.25)),
                                        drivetrain.followPath(paths.firstShootIntakeFirstRow),
                                        instant(() -> drivetrain.follower.setMaxPower(1))
                                ),
                                sequential(
                                        waitUntil(flywheel::atTarget),
                                        shoot()
                                )
                        ),
                        drivetrain.followPath(paths.toSecondShoot),
                        waitMs(200),
                        shoot(),
                        drivetrain.followPath(paths.intakeSecondRow1),
                        drivetrain.followPath(paths.intakeSecondRow2),
                        drivetrain.followPath(paths.toThirdShoot),
                        shoot(),
                        drivetrain.followPath(paths.toGateIntake),
                        drivetrain.followPath(paths.toShoot),
                        shoot(),
                        intake.off(),
                        instant(flywheel::off)
                )
        );
    }

    @Override
    public void loop() {
        wrapLoop(() -> {
            shootingController.prepareForLocation(
                    drivetrain.follower.getPose(),
                    drivetrain.follower.getVelocity(),
                    drivetrain.follower.getAngularVelocity(),
                    alliance()
            );
        });
    }

    public static class Paths {
        public final PathChain firstShootIntakeFirstRow;
        public final PathChain toSecondShoot;
        public final PathChain intakeSecondRow1;
        public final PathChain intakeSecondRow2;
        public final PathChain toThirdShoot;
        public final PathChain toGateIntake;
        public final PathChain toShoot;

        public Paths(Follower follower) {
            firstShootIntakeFirstRow = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(116, 127.5),
                                    new Pose(86, 104),
                                    new Pose(83, 82),
                                    new Pose(106, 82)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addPath(
                            new BezierLine(
                                    new Pose(106, 82),
                                    new Pose(122, 82)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            toSecondShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(122, 82),
                                    new Pose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .build();

            intakeSecondRow1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(80, 82),
                                    new Pose(105, 58.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-50), 0)
                    .build();

            intakeSecondRow2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(105, 58.5),
                                    new Pose(128, 58.5)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .build();

            toThirdShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128, 58.5),
                                    new Pose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-30))
                    .build();

            toGateIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(80, 82),
                                    new Pose(130, 57)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(45))
                    .build();

            toShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(130, 57),
                                    new Pose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-30))
                    .build();
        }
    }
}
