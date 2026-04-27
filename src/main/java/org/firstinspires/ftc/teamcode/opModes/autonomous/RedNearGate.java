package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.control.ShootingController;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

@Autonomous
public class RedNearGate extends RobotOpMode {

    private static final Vector zero = new Vector();
    private Paths paths;
    private ShootingController shootingController;
    private Pose shootingLocation;
    private boolean sotm = true;

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
                waitMs(1500),
                instant(blocker::block)
        );
    }

    @Override
    public void start() {
        schedule(
                sequential(
                        intake.on(),
//                        prepareToShoot(paths.toFirstShoot),
                        instant(flywheel::on),
                        waitUntil(() -> flywheel.getVelocity() >= flywheel.getTarget() * 0.5),
                        instant(() -> sequential(
                                waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                shoot(),
                                instant(() -> drivetrain.follower.setMaxPower(1))
                        ).schedule()),
                        drivetrain.followPathSotm(paths.toFirstShoot),
                        waitMs(250),
                        drivetrain.followPath(paths.intakeFirstRow),
//                        prepareToShoot(paths.toSecondShoot),
//                        instant(() -> sotm = false),
                        instant(() -> sequential(
                                waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                shoot(),
                                instant(() -> drivetrain.follower.setMaxPower(1))
                        ).schedule()),
                        drivetrain.followPathSotm(paths.toSecondShoot),
                        waitMs(250),
                        drivetrain.followPathSotm(paths.intakeSecondRow1),
                        instant(() -> drivetrain.follower.setMaxPower(0.8)),
                        drivetrain.followPath(paths.intakeSecondRow2),
                        instant(() -> drivetrain.follower.setMaxPower(1)),
//                        prepareToShoot(paths.toThirdShoot),
                        instant(() -> sequential(
                                waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                shoot(),
                                instant(() -> drivetrain.follower.setMaxPower(1))
                        ).schedule()),
                        drivetrain.followPathSotm(paths.toThirdShoot),
                        waitMs(250),
                        repeat(
                                sequential(
                                        drivetrain.followPath(paths.toGateIntake),
                                        race(
                                                waitUntil(artifactSensor::hasThree),
                                                waitMs(3000)
                                        ),
//                                        prepareToShoot(paths.toShoot),
                                        instant(() -> sequential(
                                                waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                                shoot(),
                                                instant(() -> drivetrain.follower.setMaxPower(1))
                                        ).schedule()),
                                        drivetrain.followPathSotm(paths.toShoot),
                                        waitMs(250)
                                ),
                                4
                        ),
                        instant(flywheel::off),
                        intake.off()
                )
        );
    }

    @Override
    public void loop() {
        wrapLoop(() -> {
            shootingController.prepareForLocation(
                    drivetrain.follower.getPose(),
                    sotm ? drivetrain.follower.getVelocity() : zero,
                    drivetrain.follower.getAngularVelocity(),
                    alliance()
            );

//            if (intake.getCurrent() > 6.5) intake.shortReverse().schedule();
        });
    }

    private Command prepareToShoot(PathChain path) {
        return instant(() -> shootingLocation = path.endPose().withHeading(path.getFinalHeadingGoal()));
    }

    public static class Paths {
        public final PathChain toFirstShoot;
        public final PathChain intakeFirstRow;
        public final PathChain toSecondShoot;
        public final PathChain intakeSecondRow1;
        public final PathChain intakeSecondRow2;
        public final PathChain toThirdShoot;
        public final PathChain toGateIntake;
        public final PathChain toShoot;

        public Paths(Follower follower) {
            toFirstShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(116, 127.5),
                                    new Pose(104, 116)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(0.7, () -> follower.setMaxPower(0.5))
                    .build();

            intakeFirstRow = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(104, 116),
                                    new Pose(81, 82),
                                    new Pose(122, 82)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(0)
                    .setTValueConstraint(0.95)
                    .build();

            toSecondShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(122, 82),
                                    new Pose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .addParametricCallback(0.65, () -> follower.setMaxPower(0.5))
                    .build();

            intakeSecondRow1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(80, 82),
                                    new Pose(103.5, 55.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-50), 0)
                    .build();

            intakeSecondRow2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(103.5, 55.5),
                                    new Pose(122, 55.5)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .build();

            toThirdShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(122, 55.5),
                                    new Pose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-30))
                    .addParametricCallback(0.65, () -> follower.setMaxPower(0.5))
                    .build();

            toGateIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(80, 82),
                                    new Pose(129.5, 57)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-30), Math.toRadians(22))
                    .setHeadingConstraint(Math.toRadians(10))
                    .setTimeoutConstraint(0)
                    .build();

            toShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(129.5, 57),
                                    new Pose(125, 50),
                                    new Pose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-30))
                    .addParametricCallback(0.6, () -> follower.setMaxPower(0.4))
                    .build();
        }
    }
}
