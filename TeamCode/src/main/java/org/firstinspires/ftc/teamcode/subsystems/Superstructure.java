package org.firstinspires.ftc.teamcode.subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.DoubleSupplier;

@Config
public class Superstructure implements Subsystem {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Servo wrist;
    private Servo claw;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.2, 0.0, 0.01);
    public static double kG = 0.15;

    private static PIDFController controller
            = new PIDFController(pidCoefficients, 0.0, 0.0, 0.0, (v, a) -> kG);

    public static double clawOpenPos = 1.0;
    public static double clawClosedPos = 0.6;

    public static double wristIntakePos = 0.75;
    public static double wristKnockedOverPos = 0.3;
    public static double wristDepositingPos = 1.0;

    public static double heightOffset = 1.0;
    public static double[] coneIntakeHeights = new double[] {1, 4.3, 5.6, 6.6, 7.8};
    public static double intakeHeightKnockedOver = 5.0;

    private static int intakeHeight = 0;

    public static double depositLowHeight = 13.5;
    public static double depositMidHeight = 23.5;
    public static double depositHighHeight = 33.5;

    public static double depositChangeHeight = 3.0;

    public enum States {
        INTAKING,
        KNOCKED_OVER_INTAKING,
        DEPOSIT_HEIGHT,
        DEPOSITING,
        WAITING_FOR_CLEARANCE
    }

    private double manualPower = 0.0;
    private boolean manualControl = false;

    private States state = States.INTAKING;

    public boolean hasClearance = false;

    public boolean manualClawOpen = false;

    public Superstructure(HardwareMap hardwareMap, boolean isAuto) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        wrist = hardwareMap.get(Servo.class, "wristServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        if (isAuto) {
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            wrist.setPosition(wristIntakePos);
            claw.setPosition(clawClosedPos);
            controller.reset();
            controller.setTargetPosition(0);
        }
    }

    public void intakeKnockedOver() {
        state = States.KNOCKED_OVER_INTAKING;
        controller.reset();
        controller.setTargetPosition(intakeHeightKnockedOver);
    }

    public void depositAtHeight(double height) {
        controller.setTargetPosition(height);
        state = States.DEPOSIT_HEIGHT;
    }

    public void changeOffset(double amount) {
        heightOffset += amount;
    }

    public void increaseIntakeHeight() {
        if (state != States.INTAKING) {
            setIntakeHeight(0);
        }
        else {
            setIntakeHeight(intakeHeight + 1);
        }
    }

    public void decreaseIntakeHeight() {
        if (state != States.INTAKING) {
            setIntakeHeight(0);
        }
        else {
            setIntakeHeight(intakeHeight - 1);
        }
    }

    private void setIntakeHeight(int height) {
        intakeHeight = Math.abs(height % coneIntakeHeights.length); // -1 % 5 = -1 in java
        controller.reset();
        controller.setTargetPosition(coneIntakeHeights[intakeHeight]);

        state = States.INTAKING;
    }

    public void deposit() {
        controller.reset();
        controller.setTargetPosition(getCurrentPosition() - depositChangeHeight);
        state = States.WAITING_FOR_CLEARANCE;
    }

    public Command manualPower(DoubleSupplier powerSupplier) {
        return new FunctionalCommand(
                () -> { manualControl = true; },
                () -> { manualPower = powerSupplier.getAsDouble(); },
                (isInterrupted) -> {
                    controller.setTargetPosition(getCurrentPosition());
                    controller.reset();
                    manualControl = false;
                    },
                () -> false,
                this
        );
    }

    @Override
    public void periodic() {
        double power = manualControl
                ? manualPower + kG
                : controller.update(getCurrentPosition());

        double clawPos = clawClosedPos;

        switch (state) {
            case INTAKING:
                wrist.setPosition(wristIntakePos);
                break;
            case KNOCKED_OVER_INTAKING:
                wrist.setPosition(wristKnockedOverPos);
                break;
            case DEPOSIT_HEIGHT:
                wrist.setPosition(wristDepositingPos);
                break;
            case DEPOSITING:
                wrist.setPosition(wristDepositingPos);

                if (!isBusy()) {
                    clawPos = clawOpenPos;
                    state = States.WAITING_FOR_CLEARANCE;
                }
                break;
            case WAITING_FOR_CLEARANCE:
                clawPos = clawOpenPos;

                if (hasClearance) {
                    setIntakeHeight(0);
                }

                break;
        }

        claw.setPosition(manualClawOpen ? clawOpenPos : clawPos);

        leftMotor.setPower(power);
        rightMotor.setPower(power);

        FtcDashboard.getInstance().getTelemetry().addData("Setpoint", controller.getTargetPosition());
        FtcDashboard.getInstance().getTelemetry().addData("Current", getCurrentPosition());

        FtcDashboard.getInstance().getTelemetry().addData("Power", power);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    private double radius = 38 / 25.4 / 2;
    private double gearRatio = 5.2;

    private boolean isBusy() {
        return Math.abs(getCurrentPosition() - controller.getTargetPosition()) < 0.2;
    }

    private double getCurrentPosition() {
        return ticksToInches(leftMotor.getCurrentPosition()) + heightOffset;
    }

    private double ticksToInches(int ticks) {
        return radius * 2 * Math.PI * ticks / (28 * gearRatio);
    }

    private int inchesToTicks(double inches) {
        return (int) (inches * (28 * gearRatio) / (radius * 2 * Math.PI));
    }
}
