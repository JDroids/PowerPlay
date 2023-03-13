package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.wpilib.Commands.instant;
import static org.firstinspires.ftc.teamcode.wpilib.Commands.parallel;
import static org.firstinspires.ftc.teamcode.wpilib.Commands.sequence;
import static org.firstinspires.ftc.teamcode.wpilib.Commands.waitSeconds;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class LeftPreloadAuto extends OpModeTemplate {
    @Override
    public void initialize() {
        initHardware(true);

        Pose2d initialPose = pose(vec(35.5, -63), rad(90));
        drive.setPoseEstimate(initialPose);

        TrajectorySequence driveToMidJunction =
                drive.trajectorySequenceBuilder(initialPose)
                        .forward(27.0)
                        .turn(rad(-45))
                        .forward(10)
                        .build();

        TrajectorySequence parkLeft =
                drive.trajectorySequenceBuilder(driveToMidJunction.end())
                        .back(10)
                        .turn(rad(-45))
                        .back(22)
                        .turn(rad(90))
                        .forward(10)
                        .build();

        TrajectorySequence parkCenter =
                drive.trajectorySequenceBuilder(driveToMidJunction.end())
                        .back(10)
                        .turn(rad(45))
                        .forward(10)
                        .build();

        TrajectorySequence parkRight =
                drive.trajectorySequenceBuilder(driveToMidJunction.end())
                        .back(10)
                        .turn(rad(-45))
                        .forward(25)
                        .turn(rad(90))
                        .forward(10)
                        .build();

        int id = waitForStartAndReturnAprilTagID();

        TrajectorySequence parkTrajectorySequence =
                new TrajectorySequence[]{parkLeft, parkCenter, parkRight}[id - 1];

        schedule(
                sequence(
                        waitSeconds(10),
                        parallel(
                                followTrajectorySequence(driveToMidJunction),
                                instant(() -> superstructure.depositAtHeight(
                                        Superstructure.depositMidHeight))
                        ),
                        waitSeconds(1.0),
                        instant(() -> superstructure.deposit()),
                        waitSeconds(1.0),
                        parallel(
                                followTrajectorySequence(parkTrajectorySequence),
                                sequence(
                                        waitSeconds(2.0),
                                        instant(() -> superstructure.setIntakeHeight(0))
                                )
                        ),
                        instant(this::requestOpModeStop)
                )
        );
    }
}
