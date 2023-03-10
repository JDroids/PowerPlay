package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Superstructure;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

abstract public class OpModeTemplate extends CommandOpMode {
    protected SampleMecanumDrive drive;
    protected Superstructure superstructure;
    private List<LynxModule> hubs;

    protected void initHardware(boolean isAuto) {
        drive = new SampleMecanumDrive(hardwareMap);
        superstructure = new Superstructure(hardwareMap, isAuto);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        register(superstructure);
    }

    @Override
    public void run() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        drive.update();

        super.run();
    }

    protected int waitForStartAndReturnAprilTagID() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        AprilTagDetectionPipeline pipeline = new AprilTagDetectionPipeline(1.0, 1.0, 1.0, 1.0, 1.0);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        int id = 2;

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                telemetry.addData("detected id", currentDetections.get(0).id);

                id = currentDetections.get(0).id;
            }
            else {
                telemetry.addData("detected id", "Nothing Found :(");
                telemetry.addData("last detected", id);
            }

            telemetry.update();
        }

        return id;
    }

    protected Command followTrajectorySequence(TrajectorySequence trajectorySequence) {
        return new FunctionalCommand(
                () -> drive.followTrajectorySequenceAsync(trajectorySequence),
                () -> {},
                (interrupted) -> {},
                () -> !drive.isBusy());
    }
    
    // utility aliases 
    protected Pose2d pose(Vector2d vec, double heading) {
        return new Pose2d(vec, heading);
    }
    
    protected Vector2d vec(double x, double y) {
        return new Vector2d(x, y);
    }

    protected double rad(double degrees) {
        return Math.toRadians(degrees);
    }
}
