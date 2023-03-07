package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class Updater implements Subsystem {
    private SampleMecanumDrive drive;
    List<LynxModule> hubs;

    public Updater(HardwareMap hardwareMap, SampleMecanumDrive drive) {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        this.drive = drive;
    }

    @Override
    public void periodic() {
        drive.update();

        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
