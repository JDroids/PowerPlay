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
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.function.DoubleSupplier;

@Config
public class Superstructure implements Subsystem {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Servo wrist;
    private Servo claw;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.15, 0.0, 0.0);
    public static double kG = 0.05;

    private static PIDFController controller
            = new PIDFController(pidCoefficients, 0.0, 0.0, 0.0, (v, a) -> kG);

    public static double clawOpenPos = 0.0;
    public static double clawOpenKnockedOverPos = 0.35;
    public static double clawClosedPos = 0.55;

    public static double wristIntakePos = 0.58;
    public static double wristKnockedOverPos = 0.0;
    public static double wristDepositingPos = 0.9;

    public static double heightOffset = 0.0;

    public static double coneIntakeHeightGround = 1;
    public static double coneIntakeHeightStack1 = 3;
    public static double coneIntakeHeightStack2 = 4;
    public static double coneIntakeHeightStack3 = 5.15;
    public static double coneIntakeHeightStack4 = 6.4;
    private static double[] coneIntakeHeights = new double[] {
            coneIntakeHeightGround,
            coneIntakeHeightStack1,
            coneIntakeHeightStack2,
            coneIntakeHeightStack3,
            coneIntakeHeightStack4
    };

    private static int intakeHeight = 2;
    public static double intakeHeightKnockedOver = 7.0;

    public static double depositLowHeight = 10;
    public static double depositMidHeight = 20;
    public static double depositHighHeight = 31.5;

    public static double depositChangeHeight = 5.0;

    public enum States {
        WAITING,
        INTAKING,
        KNOCKED_OVER_INTAKING,
        DEPOSIT_HEIGHT,
        DEPOSITING,
        WAITING_FOR_CLEARANCE,
        GOING_DOWN
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
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            wrist.setPosition(1);
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
            state = States.INTAKING;
        }
        else {
            setIntakeHeight(intakeHeight + 1);
            state = States.INTAKING;
        }
    }

    public void decreaseIntakeHeight() {
        if (state != States.INTAKING) {
            setIntakeHeight(0);
            state = States.INTAKING;
        }
        else {
            setIntakeHeight(intakeHeight - 1);
            state = States.INTAKING;
        }
    }

    public void setIntakeHeight(int height) {
        int levels = coneIntakeHeights.length;;
        intakeHeight = Math.abs((height % levels + levels) % levels);
        // -1 % 5 = -1 in java, so double mod to wrap around properly
        controller.reset();
        controller.setTargetPosition(coneIntakeHeights[intakeHeight]);
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
            case WAITING:
                break;
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
                wrist.setPosition(wristIntakePos);

                if (hasClearance) {
                    setIntakeHeight(0);
                    state = States.GOING_DOWN;
                }

                break;
            case GOING_DOWN:
                clawPos = clawOpenPos;

                if (!isBusy()) {
                    state = States.INTAKING;
                }
                break;
        }

        if (state != States.KNOCKED_OVER_INTAKING) {
            claw.setPosition(manualClawOpen ? clawOpenPos : clawPos);
        }
        else {
            claw.setPosition(manualClawOpen ? clawOpenKnockedOverPos : clawPos);
        }

        double clampedPower = Math.max(-0.6, power);

        leftMotor.setPower(clampedPower);
        rightMotor.setPower(clampedPower);


        FtcDashboard.getInstance().getTelemetry().addData("State", state);
        FtcDashboard.getInstance().getTelemetry().addData("Setpoint", controller.getTargetPosition());
        FtcDashboard.getInstance().getTelemetry().addData("Current", getCurrentPosition());

        FtcDashboard.getInstance().getTelemetry().addData("Power", power);
        FtcDashboard.getInstance().getTelemetry().update();
    }

    private double radius = 38 / 25.4 / 2;
    private double gearRatio = 5.2;

    private boolean isBusy() {
        return Math.abs(getCurrentPosition() - controller.getTargetPosition()) > 1.0;
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
