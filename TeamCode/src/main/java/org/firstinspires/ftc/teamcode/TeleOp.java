package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpModeTemplate {

    @Override
    public void initialize() {
        initHardware(false);

        new Trigger(() -> gamepad2.cross).whenActive(superstructure::deposit);

        new Trigger(() -> gamepad2.square).whenActive(
                () -> superstructure.depositAtHeight(Superstructure.depositLowHeight));
        new Trigger(() -> gamepad2.triangle).whenActive(
                () -> superstructure.depositAtHeight(Superstructure.depositMidHeight));
        new Trigger(() -> gamepad2.circle).whenActive(
                () -> superstructure.depositAtHeight(Superstructure.depositHighHeight));

        new Trigger(() -> gamepad2.dpad_up).whenActive(superstructure::increaseIntakeHeight);
        new Trigger(() -> gamepad2.dpad_down).whenActive(superstructure::decreaseIntakeHeight);

        new Trigger(() -> gamepad2.left_bumper).whenActive(superstructure::intakeKnockedOver);

        new Trigger(() -> gamepad2.dpad_left).whenActive(() -> superstructure.changeOffset(-0.2));
        new Trigger(() -> gamepad2.dpad_left).whenActive(() -> superstructure.changeOffset(0.2));

        new Trigger(
                () -> Math.abs(gamepad2.right_trigger - gamepad2.left_trigger) > 0.05
        ).whileActiveOnce(
                superstructure.manualPower(() -> gamepad2.right_trigger - gamepad2.left_trigger));

        schedule(new InstantCommand(() -> superstructure.setIntakeHeight(0)));
        // go to intaking position when teleop starts
    }

    @Override
    public void run() {
        super.run();

        Pose2d drivePower = new Pose2d(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x);
        drive.setDrivePower(drivePower);
        superstructure.hasClearance = drivePower.vec().norm() > 0.1;

        superstructure.manualClawOpen = gamepad2.right_bumper;
    }
}
