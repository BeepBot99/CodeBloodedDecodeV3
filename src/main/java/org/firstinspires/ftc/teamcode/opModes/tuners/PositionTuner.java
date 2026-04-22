package org.firstinspires.ftc.teamcode.opModes.tuners;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.RobotOpMode;

@TeleOp(name = "Position Tuner", group = "Tuners")
public class PositionTuner extends RobotOpMode {
    @Override
    public void loop() {
        wrapLoop(() -> {
            drivetrain.arcadeDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    alliance()
            );
        });
    }
}
