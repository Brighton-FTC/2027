package org.firstinspires.ftc.teamcode.transfer;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Servo.ColourServo;

public class TransferIntakeTeleop {

    // ---------- tune these ----------
    private static final double TICKS_PER_REV = 537.7;      // TODO goBILDA 312rpm = 537.7, 435rpm = 384.5
    private static final double PITCH_DIAMETER_MM = 36.0;
    private static final double STEP_INCHES = 3.8;
    private static final int STEP_TICKS =
            (int) Math.round(STEP_INCHES * 25.4 / (Math.PI * PITCH_DIAMETER_MM) * TICKS_PER_REV); // ~459 ticks

    private static final double TRANSFER_POWER = 1.0;        // max power while moving to a position
    private static final double POSITION_KP = 0.05;          // TODO tune: higher = snappier, too high = jitter
    private static final double POSITION_TOLERANCE = 10;     // ticks
    private static final double INTAKE_POWER = 1.0;
    private static final double FEEDER_POWER = 1.0;          // CR servo full speed
    private static final double HOLD_SECONDS = 0.3;          // how long before a press counts as a hold
    private static final double INTAKE_DOWN = 0.0;           // TODO tune
    private static final double INTAKE_UP = 0.6;             // TODO tune

    // ---------- hardware ----------
    private final Motor transfer;
    private final CRServo feeder;
    private final DcMotor intake;
    private final Servo intakeLift;
    private final ColourServo colourServo;

    // ---------- state ----------
    private boolean intakeOn = false;
    private boolean ballWasSeen = false;
    private int transferTarget = 0;
    private boolean positionMode = true;   // true = PositionControl, false = RawPower
    private double manualPower = 0;
    private boolean prevFiring = false, prevIntake = false;
    private final ElapsedTime fireTimer = new ElapsedTime();

    public TransferIntakeTeleop(HardwareMap hardwareMap, String transferID, String feederID, String intakeID,
                                String intakeLiftID, String compressionID, String sensorID) {
        transfer = new Motor(hardwareMap, transferID);
        feeder = hardwareMap.get(CRServo.class, feederID);
        intake = hardwareMap.get(DcMotor.class, intakeID);
        intakeLift = hardwareMap.get(Servo.class, intakeLiftID);
        colourServo = new ColourServo(hardwareMap, compressionID, sensorID);

        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.resetEncoder();
        transfer.setPositionCoefficient(POSITION_KP);
        transfer.setPositionTolerance(POSITION_TOLERANCE);
        holdTransferHere();

        intakeLift.setPosition(INTAKE_UP);
    }

    /**
     * Call once per loop.
     * @param shooterOn    whether the shooter is currently running
     * @param fireButton   held = true (tap steps once, hold keeps feeding)
     * @param intakeButton toggles the intake on press
     * @param flushButton  held = flush
     */
    public void update(boolean shooterOn, boolean fireButton, boolean intakeButton,
                       boolean flushButton, Telemetry telemetry) {

        boolean firing = shooterOn && fireButton && !flushButton;

        // intake toggle
        if (intakeButton && !prevIntake) {
            intakeOn = !intakeOn;
            ballWasSeen = false;
        }

        if (flushButton) {
            // lift intake, transfer backwards full speed
            intakeOn = false;
            ballWasSeen = false;
            intakeLift.setPosition(INTAKE_UP);
            intake.setPower(0);
            feeder.setPower(0);
            runTransferManual(-1.0);

        } else if (firing) {
            feeder.setPower(FEEDER_POWER);
            if (!prevFiring) {
                fireTimer.reset();
                stepTransfer();                                   // tap: one 3.8 in step
            } else if (fireTimer.seconds() > HOLD_SECONDS) {
                runTransferManual(1.0);                           // hold: keep feeding
            }

        } else {
            feeder.setPower(0);
            if (!positionMode) {
                holdTransferHere();
            }

            if (intakeOn) {
                intakeLift.setPosition(INTAKE_DOWN);
                intake.setPower(INTAKE_POWER);

                // new ball at the sensor -> advance transfer one step
                ColourServo.detectedColor colour = colourServo.update(telemetry);
                boolean ballSeen = colour != ColourServo.detectedColor.UNKNOWN;
                if (ballSeen && !ballWasSeen) stepTransfer();
                ballWasSeen = ballSeen;
            } else {
                intakeLift.setPosition(INTAKE_UP);
                intake.setPower(0);
            }
        }

        // FTCLib only runs the position loop when set() is called, so drive it every loop
        driveTransfer();

        prevFiring = firing;
        prevIntake = intakeButton;

        telemetry.addData("intake", intakeOn ? "ON" : "off");
        telemetry.addData("transfer mode", positionMode ? "position" : "manual");
        telemetry.addData("transfer pos", transfer.getCurrentPosition());
        telemetry.addData("transfer target", transferTarget);
        telemetry.addData("transfer at target", transfer.atTargetPosition());
    }

    public void stop() {
        transfer.stopMotor();
        intake.setPower(0);
        feeder.setPower(0);
    }

    // send power to the transfer based on the current mode
    private void driveTransfer() {
        if (positionMode) {
            transfer.set(TRANSFER_POWER);
        } else {
            transfer.set(manualPower);
        }
    }

    // move the transfer forward by exactly one ball (3.8 in)
    private void stepTransfer() {
        if (!positionMode) holdTransferHere();
        transferTarget += STEP_TICKS;
        transfer.setTargetPosition(transferTarget);
    }

    // free-running transfer (hold-to-fire and flush)
    private void runTransferManual(double power) {
        if (positionMode) {
            transfer.setRunMode(Motor.RunMode.RawPower);
            positionMode = false;
        }
        manualPower = power;
    }

    // lock the transfer wherever it is right now
    private void holdTransferHere() {
        transfer.setRunMode(Motor.RunMode.PositionControl);
        positionMode = true;
        transferTarget = transfer.getCurrentPosition();
        transfer.setTargetPosition(transferTarget);
    }
}