package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class ParkAuto extends OpModeTemplate {
    @Override
    public void initialize() {
        initHardware(true);

        Pose2d initialPose = pose(vec(35.5, -63), rad(90));
        drive.setPoseEstimate(initialPose);

        TrajectorySequence parkLeft =
                drive.trajectorySequenceBuilder(initialPose)
                        .forward(28.0)
                        .turn(rad(90))
                        .forward(24)
                        .turn(rad(-90))
                        .forward(10)
                        .build();

        TrajectorySequence parkCenter =
                drive.trajectorySequenceBuilder(initialPose)
                        .forward(38.0)
                        .build();

        TrajectorySequence parkRight =
                drive.trajectorySequenceBuilder(initialPose)
                        .forward(28.0)
                        .turn(rad(90))
                        .back(24)
                        .turn(rad(-90))
                        .forward(10)
                        .build();

        int id = waitForStartAndReturnAprilTagID();

        TrajectorySequence parkTrajectorySequence =
                new TrajectorySequence[]{parkLeft, parkCenter, parkRight}[id - 1];

        schedule(followTrajectorySequence(parkTrajectorySequence));
    }
}
