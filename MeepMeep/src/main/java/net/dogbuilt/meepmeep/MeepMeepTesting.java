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
                    .turn(rad(45)) // will need changes depending on park location
                    .back(23)
                    .turn(rad(-90))
                    .forward(10)
                    .build();
    private static AddTrajectorySequenceCallback cycling = drive ->
            drive.trajectorySequenceBuilder(pose(vec(35.5, -63), rad(90)))
                    .forward(51.0)

                    // drop preload
                    .turn(rad(135))
                    .forward(6)
                    .back(6)
                    .turn(rad(135))

                    // cycle one
                    .forward(22)
                    .back(22)
                    .turn(rad(-135))
                    .forward(6)
                    .back(6)
                    .turn(rad(135))

                    // cycle two
                    .forward(22)
                    .back(22)
                    .turn(rad(-135))
                    .forward(6)
                    .back(6)
                    .turn(rad(135))

                    // cycle three
                    .forward(22)
                    .back(22)
                    .turn(rad(-135))
                    .forward(6)
                    .back(6)
                    .turn(rad(135))

                    .build();

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(13, 15)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(80, 40, Math.toRadians(360), Math.toRadians(270), 14)
                .followTrajectorySequence(preload);

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