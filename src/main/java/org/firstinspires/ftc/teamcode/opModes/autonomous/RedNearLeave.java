package org.firstinspires.ftc.teamcode.opModes.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

import static com.pedropathing.ivy.Scheduler.schedule;

@Autonomous(name = "Red Near Leave", group = "Autonomous")
public class RedNearLeave extends RobotOpMode {
    private Paths paths;

    @Override
    public void init() {
        super.init();

        drivetrain.follower.setStartingPose(new Pose(116, 127.5, Math.toRadians(-142.5)));
        turret.setStartingAngle(0);
        blocker.block();
        turret.on();
        hood.enable();

        paths = new Paths(drivetrain.follower);
        setAlliance(Alliance.RED);
    }

    @Override
    public void start() {
        schedule(drivetrain.followPath(paths.leave));
    }

    @Override
    public void loop() {
        wrapLoop(() -> {
        });
    }

    public static class Paths {
        public final PathChain leave;

        public Paths(Follower follower) {
            leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    new Pose(116, 172.5),
                                    new Pose(100, 120)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(-142.5))
                    .build();
        }
    }
}
