package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;

@Autonomous
public class LeftPreloadAuto extends LinearOpMode {
    private DcMotorEx motorFrontLeft;
    private DcMotorEx motorBackLeft;
    private DcMotorEx motorFrontRight;
    private DcMotorEx motorBackRight;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "backRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx lift = hardwareMap.get(DcMotorEx.class, "liftMotor");
        lift.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        Servo claw = hardwareMap.get(Servo.class, "clawServo");

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

        claw.setPosition(0.1);

        sleep(500);

        lift.setTargetPosition(-300);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.5);

        while (lift.isBusy()) {}

        runToPosition(59, 59, 0.3);

        lift.setTargetPosition(-4400);
        runToPosition(10, -10, 0.3);
        runToPosition(9, 9, 0.3);
        while (lift.isBusy()) {}

        lift.setTargetPosition(-3600);
        while (lift.isBusy()) {}
        claw.setPosition(0.2);

        sleep(1000);
        runToPosition(-10, -10, 0.3);

        claw.setPosition(0.1);
        lift.setTargetPosition(-500);

        runToPosition(10, -10, 0.3);

        if (id == 1) {
            runToPosition(-28, -28, 0.3);
        }
        else if (id == 3) {
            runToPosition(26, 26, 0.3);
        }
    }

    private void runToPosition(double leftInches, double rightInches, double power) {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setTargetPosition(inchesToTicks(leftInches));
        motorFrontRight.setTargetPosition(inchesToTicks(rightInches));
        motorBackLeft.setTargetPosition(inchesToTicks(leftInches));
        motorBackRight.setTargetPosition(inchesToTicks(rightInches));

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        while (motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy()) {}

        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private int inchesToTicks(double inches) {
        return (int) ((inches / (Math.PI *  (96 / 25.4))) * 28 * 15);
    }
}
