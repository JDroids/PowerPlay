package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Config
public class Superstructure implements Subsystem {
    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;
    private Servo wrist;
    private Servo claw;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.0, 0.0, 0.0);
    public static double kG = 0.0;

    private static PIDFController controller
            = new PIDFController(pidCoefficients, 0.0, 0.0, 0.0, (v, a) -> kG);

    public static double clawOpenPos = 0.0;
    public static double clawClosedPos = 0.0;

    public static double wristIntakePos = 0.0;
    public static double wristKnockedOverPos = 0.0;
    public static double wristDepositingPos = 0.0;

    public static double heightOffset = 2.0;

    public static double intakeHeightLevel0 = 3.0;
    public static double coneHeight = 1.326;
    public static double intakeHeightKnockedOver = 5.0;

    private static int intakeHeight = 0;

    public static double depositLowHeight = 13.5;
    public static double depositMidHeight = 23.5;
    public static double depositHighHeight = 33.5;

    public static double depositChangeHeight = 3.0;

    public enum States {
        INTAKING,
        KNOCKED_OVER_INTAKING,
        DEPOSITING,
        WAITING_FOR_CLEARANCE
    }

    private double manualPower = 0.0;
    private boolean manualControl = false;

    private States state = States.INTAKING;

    private BooleanSupplier hasClearance = () -> false;

    public boolean manualClawOpen = false;

    public Superstructure(HardwareMap hardwareMap, boolean isAuto) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        wrist = hardwareMap.get(Servo.class, "wristServo");
        claw = hardwareMap.get(Servo.class, "clawServo");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if (isAuto) {
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
        state = States.DEPOSITING;
    }

    public void changeOffset(double amount) {
        heightOffset += amount;
    }

    public void increaseIntakeHeight() {
        if (state != States.INTAKING) {
            state = States.INTAKING;
            intakeHeight = 5;
            return;
        }

        intakeHeight = (intakeHeight + 1) % 5;
    }

    public void decreaseIntakeHeight() {
        if (state != States.INTAKING) {
            state = States.INTAKING;
            intakeHeight = 0;
            return;
        }

        intakeHeight = (intakeHeight - 1) % 5;
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
            case DEPOSITING:
                wrist.setPosition(wristDepositingPos);
                break;
            case WAITING_FOR_CLEARANCE:
                wrist.setPosition(wristDepositingPos);

                if (!isBusy()) {
                    clawPos = clawOpenPos;

                    if (hasClearance.getAsBoolean()) {
                        controller.reset();
                        decreaseIntakeHeight(); // reset to intake height 0
                    }
                }
                break;
        }

        claw.setPosition(manualClawOpen ? clawOpenPos : clawPos);

        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    private double radius = 38 / 25.4 / 2;
    private double gearRatio = 13.7;

    private boolean isBusy() {
        return Math.abs(getCurrentPosition() - controller.getTargetPosition()) < 0.2;
    }

    private double getCurrentPosition() {
        return ticksToInches(leftMotor.getCurrentPosition()) - heightOffset;
    }

    private double ticksToInches(int ticks) {
        return radius * 2 * Math.PI * ticks / (28 * gearRatio);
    }

    private int inchesToTicks(double inches) {
        return (int) (inches * (28 * gearRatio) / (radius * 2 * Math.PI));
    }
}
