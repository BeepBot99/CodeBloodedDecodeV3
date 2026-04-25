package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.*;

import java.util.List;

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

    private List<LynxModule> hubs;
    private final ElapsedTime loopTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

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

        hubs = hardwareMap.getAll(LynxModule.class);

        hubs.forEach(hub -> hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL));
    }

    @Override
    public void init_loop() {
        context.telemetry.addData("Robot/loop time", loopTimer.milliseconds());
        loopTimer.reset();
        context.telemetry.addData("Robot/alliance", alliance());

        Scheduler.execute();
        context.telemetry.update();
    }

    protected final void wrapLoop(Runnable runnable) {
        context.telemetry.addData("Robot/loop time", loopTimer.milliseconds());
        loopTimer.reset();
        hubs.forEach(LynxModule::clearBulkCache);
        drivetrain.update();
        context.telemetry.addData("Robot/alliance", alliance());

        runnable.run();

        Scheduler.execute();
        context.telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.close();
    }
}
