package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.*;

import static com.pedropathing.ivy.Scheduler.schedule;

public abstract class RobotOpMode extends OpMode {
    private static Alliance currentAlliance = Alliance.RED;
    protected Blocker blocker;
    protected Drivetrain drivetrain;
    protected Turret turret;
    protected Flywheel flywheel;
    protected Intake intake;
    protected Hood hood;
    protected Context context;

    protected static Alliance alliance() {
        return currentAlliance;
    }

    public static void setAlliance(Alliance alliance) {
        currentAlliance = alliance;
    }

    @Override
    public void init() {
        Scheduler.reset();
        context = new Context(this);

        blocker = new Blocker(context);
        drivetrain = new Drivetrain(context);
        turret = new Turret(context);
        flywheel = new Flywheel(context);
        intake = new Intake(context);
        hood = new Hood(context);

        schedule(
                blocker.periodic(),
                drivetrain.periodic(),
                turret.periodic(),
                flywheel.periodic(),
                intake.periodic(),
                hood.periodic()
        );
    }

    @Override
    public void init_loop() {
        context.telemetry.addData("Robot/alliance", alliance());

        Scheduler.execute();
        context.telemetry.update();
    }

    @Override
    public void loop() {
        context.telemetry.addData("Robot/alliance", alliance());

        Scheduler.execute();
        context.telemetry.update();
    }
}
