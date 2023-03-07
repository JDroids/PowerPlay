package org.firstinspires.ftc.teamcode.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Disabled
@Autonomous
public class Auto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo claw = hardwareMap.get(Servo.class, "clawServo");
        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "liftMotor");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // dummy values because just using identification
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
        boolean right = false;

        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.dpad_left) {
                right = false;
            }
            else if (gamepad1.dpad_right) {
                right = true;
            }

            if (right) {
                telemetry.addData("AUTO SIDE", "RIGHT");
            }
            else {
                telemetry.addData("AUTO SIDE", "LEFT");
            }

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

        // raise preload cone off of ground
        claw.setPosition(1.0);
        sleep(100);
        lift.setPower(1.0);
        lift.setTargetPosition(-3800);

        if (right) {
            // noop for now
        }
        else {
            drive.setPoseEstimate(new Pose2d(-36.0, -64.0, Math.toRadians(90.0)));
        }

        TrajectorySequence sequence = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .forward(54.0)
                .turn(Math.toRadians(-90.0))
                .forward(28.0)
                .turn(Math.toRadians(-45.0))
                .forward(5.0)
                .addTemporalMarker(() -> lift.setTargetPosition(-3400))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> claw.setPosition(0.9))
                .waitSeconds(0.2)
                .back(5.0)
                .turn(Math.toRadians(50))
                .back(id == 1 ? 52 : (id == 2 ? 28 : 4))
                .turn(Math.toRadians(-95.0))
                .build();

        drive.followTrajectorySequence(sequence);

        claw.setPosition(1.0);
        sleep(100);
        lift.setTargetPosition(0);
        while (lift.isBusy() && opModeIsActive()) {} // intentional noop
    }
}
