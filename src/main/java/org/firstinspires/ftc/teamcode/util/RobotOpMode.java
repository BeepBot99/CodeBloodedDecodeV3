package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.*;

import static com.pedropathing.ivy.Scheduler.schedule;

public abstract class RobotOpMode extends OpMode {
    protected Blocker blocker;
    protected Drivetrain drivetrain;
    protected Turret turret;
    protected Flywheel flywheel;
    protected Intake intake;
    protected Context context;

    @Override
    public void init() {
        context = new Context(this);

        blocker = new Blocker(context);
        drivetrain = new Drivetrain(context);
        turret = new Turret(context);
        flywheel = new Flywheel(context);
        intake = new Intake(context);

        schedule(
                blocker.periodic(),
                drivetrain.periodic(),
                turret.periodic(),
                flywheel.periodic(),
                intake.periodic()
        );
    }

    protected final Alliance alliance() {
        return Alliance.current;
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
