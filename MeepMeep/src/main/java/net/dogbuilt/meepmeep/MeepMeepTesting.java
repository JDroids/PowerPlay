package net.dogbuilt.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.AddTrajectorySequenceCallback;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    private static AddTrajectorySequenceCallback parkLeft = drive ->
            drive.trajectorySequenceBuilder(pose(vec(35, -63), rad(90)))
                    .forward(28.0)
                    .turn(rad(90))
                    .forward(24)
                    .turn(rad(-90))
                    .forward(10)
                    .build();

    private static AddTrajectorySequenceCallback parkCenter = drive ->
            drive.trajectorySequenceBuilder(pose(vec(35, -63), rad(90)))
                    .forward(38.0)
                    .build();

    private static AddTrajectorySequenceCallback parkRight = drive ->
            drive.trajectorySequenceBuilder(pose(vec(35.5, -63), rad(90)))
                    .forward(28.0)
                    .turn(rad(90))
                    .back(24)
                    .turn(rad(-90))
                    .forward(10)
                    .build();

    private static AddTrajectorySequenceCallback preload = drive ->
            drive.trajectorySequenceBuilder(pose(vec(35.5, -63), rad(90)))
                    .forward(27.0)
                    .turn(rad(45))
                    .forward(6)
                    .waitSeconds(2)
                    .back(6)
                    .turn(rad(-45)) // will need changes for not center
                    .forward(10)
                    .build();
    private static AddTrajectorySequenceCallback oneCycle = drive ->
            drive.trajectorySequenceBuilder(pose(vec(35.5, -63), rad(90)))
                    .forward(51.0)
                    // cycle noe
                    .turn(rad(135))
                    .forward(6)
                    .waitSeconds(1)
                    .back(6)
                    .turn(rad(135))
                    .forward(22)
                    .waitSeconds(1)
                    .back(22)
                    .turn(rad(-135))
                    .build();

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 30, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(oneCycle);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    private static Pose2d pose(Vector2d vec, double heading) {
        return new Pose2d(vec, heading);
    }

    private static Vector2d vec(double x, double y) {
        return new Vector2d(x, y);
    }

    private static double rad(double degrees) {
        return Math.toRadians(degrees);
    }
}