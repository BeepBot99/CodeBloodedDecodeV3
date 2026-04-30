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
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

@Autonomous(name = "Blue Near Gate", group = "Autonomous")
public class BlueNearGate extends RobotOpMode {

    private static final Vector zero = new Vector();
    private Paths paths;
    private ShootingController shootingController;
    private Pose shootingLocation;
    private boolean sotm = true;

    @Override
    public void init() {
        super.init();

        drivetrain.follower.setStartingPose(createPose(116, 127.5, Math.toRadians(-142.5)));
        turret.setStartingAngle(0);
        blocker.block();
        turret.on();
        hood.enable();

        paths = new Paths(drivetrain.follower);
        shootingController = new ShootingController(
                context,
                turret,
                flywheel,
                hood
        );

        setAlliance(Alliance.BLUE);
    }

    private static Pose createPose(double x, double y, double heading) {
        return new Pose(x, y, heading).mirror();
    }

    private static Pose createPose(double x, double y) {
        return createPose(x, y, 0);
    }

    private Command shoot() {
        return sequential(
                instant(blocker::unblock),
                waitMs(2000),
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
                        drivetrain.followPath(paths.intakeFirstRow),
//                        prepareToShoot(paths.toSecondShoot),
//                        instant(() -> sotm = false),
                        parallel(
                                drivetrain.followPathSotm(paths.toSecondShoot),
                                sequential(
                                        waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                        shoot()
                                )
                        ),
                        instant(() -> drivetrain.follower.setMaxPower(1)),
                        drivetrain.followPathSotm(paths.intakeSecondRow1),
                        instant(() -> drivetrain.follower.setMaxPower(0.8)),
                        drivetrain.followPathSotm(paths.intakeSecondRow2),
                        instant(() -> drivetrain.follower.setMaxPower(1)),
//                        prepareToShoot(paths.toThirdShoot),
                        parallel(
                                drivetrain.followPathSotm(paths.toThirdShoot),
                                sequential(
                                        waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                        shoot()
                                )
                        ),
                        instant(() -> drivetrain.follower.setMaxPower(1)),
                        repeat(
                                sequential(
                                        drivetrain.followPath(paths.toGateIntake),
                                        race(
                                                waitUntil(artifactSensor::hasThree),
                                                waitMs(1100)
                                        ),
//                                        prepareToShoot(paths.toShoot),
                                        parallel(
                                                drivetrain.followPathSotm(paths.toShoot),
                                                sequential(
                                                        waitUntil(() -> drivetrain.follower.getCurrentTValue() > 0.85),
                                                        shoot()
                                                )
                                        ),
                                        instant(() -> drivetrain.follower.setMaxPower(1)),
                                        waitMs(250)
                                ),
                                3
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
                                    createPose(116, 127.5),
                                    createPose(104, 116)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .addParametricCallback(0.7, () -> follower.setMaxPower(0.4))
                    .build();

            intakeFirstRow = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    createPose(104, 116),
                                    createPose(81, 82),
                                    createPose(122, 82)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .setTimeoutConstraint(0)
                    .setTValueConstraint(0.95)
                    .build();

            toSecondShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    createPose(122, 82),
                                    createPose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .addParametricCallback(0.65, () -> follower.setMaxPower(0.4))
                    .build();

            intakeSecondRow1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    createPose(80, 82),
                                    createPose(103.5, 55.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.PI - Math.toRadians(-50), Math.PI)
                    .build();

            intakeSecondRow2 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    createPose(103.5, 55.5),
                                    createPose(122, 58)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI)
                    .build();

            toThirdShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    createPose(122, 58),
                                    createPose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI - Math.toRadians(-30))
                    .addParametricCallback(0.65, () -> follower.setMaxPower(0.4))
                    .build();

            toGateIntake = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    createPose(80, 82),
                                    createPose(129.5, 57.5)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.PI - Math.toRadians(-30), Math.PI - Math.toRadians(22))
                    .setHeadingConstraint(Math.toRadians(10))
                    .setTimeoutConstraint(0)
                    .build();

            toShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    createPose(129.5, 57.5),
                                    createPose(125, 50),
                                    createPose(80, 82)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.PI - Math.toRadians(-30))
                    .addParametricCallback(0.6, () -> follower.setMaxPower(0.4))
                    .build();
        }
    }
}
