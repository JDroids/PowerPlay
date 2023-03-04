package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;

abstract public class OpModeTemplate extends CommandOpMode {
    protected SampleMecanumDrive drive;
    protected Superstructure superstructure;

    protected void initHardware(boolean isAuto) {
        drive = new SampleMecanumDrive(hardwareMap);
        superstructure = new Superstructure(hardwareMap, isAuto);

        register(superstructure);
    }
}
