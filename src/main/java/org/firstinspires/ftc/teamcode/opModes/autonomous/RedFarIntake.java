package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.control.ShootingController;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

@Autonomous(name = "Red Far Intake", group = "Autonomous")
public class RedFarIntake extends RobotOpMode {

    private static final Vector zero = new Vector();
    private Paths paths;
    private ShootingController shootingController;
    private Pose shootingLocation;
    private boolean sotm = true;

    @Override
    public void init() {
        super.init();

        drivetrain.follower.setStartingPose(new Pose(88, 9, Math.PI / 2));
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

        setAlliance(Alliance.RED);
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
                        instant(flywheel::on),
                        waitUntil(flywheel::atTarget),
                        shoot(),
                        drivetrain.followPathSotm(paths.intakeLastRow),
                        drivetrain.followPath(paths.toSecondShoot),
                        shoot(),
                        drivetrain.followPath(paths.intakeCorner),
                        drivetrain.followPath(paths.toThirdShoot),
                        shoot(),
                        repeat(
                                sequential(
                                        deadline(
                                                waitUntil(artifactSensor::hasThree),
                                                drivetrain.followPath(paths.intakeGate)
                                        ),
                                        drivetrain.followPath(paths.toShoot),
                                        shoot()
                                ),
                                3
                        )
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
        public final PathChain intakeLastRow;
        public final PathChain toSecondShoot;
        public final PathChain intakeCorner;
        public final PathChain toThirdShoot;
        public final PathChain intakeGate;
        public final PathChain toShoot;

        public Paths(Follower follower) {
            intakeLastRow = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88, 9),
                                    new Pose(88, 35),
                                    new Pose(100, 35.5),
                                    new Pose(128, 35.5)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            toSecondShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(128, 35.5),
                                    new Pose(84, 10)
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                    .build();

            intakeCorner = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(84, 10),
                                    new Pose(136, 8)
                            )
                    )
                    .setConstantHeadingInterpolation(0)
                    .addParametricCallback(0.6, () -> follower.setMaxPower(0.6))
                    .build();

            toThirdShoot = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(136, 8),
                                    new Pose(84, 10)
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                    .build();

            intakeGate = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84, 10),
                                    new Pose(137, 13.4),
                                    new Pose(137, 28.9)
                            )
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            toShoot = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(137, 28.9),
                                    new Pose(137, 13.4),
                                    new Pose(84, 10)
                            )
                    )
                    .setHeadingInterpolation(HeadingInterpolator.tangent.reverse())
                    .build();
        }
    }
}
