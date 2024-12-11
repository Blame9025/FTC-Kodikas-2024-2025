package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;

public class Intake {
    private DcMotor intakeMotor;
    private Position currentPosition = Position.DEFAULT;
    private final Object positionLock = new Object();
    KodikasRobot robot;
    public enum Position {
        DEFAULT(0),
        EXTENDED(140);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }
    Thread posThread;
    boolean killController;
    public Intake(KodikasRobot robot, DcMotor definedIntakeMotor) {
        this.intakeMotor = definedIntakeMotor;
        this.robot = robot;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(Position.DEFAULT.val);

        posThread = new Thread(new PositionPlayer());
        posThread.start();
    }

    public void setPosition(Position target) {
        synchronized (positionLock) {
            if (currentPosition != target) {
                intakeMotor.setTargetPosition(target.val);
                intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeMotor.setPower(0.5);
                currentPosition = target;
            }
        }
    }

    public int getMotorPosition(){
        return intakeMotor.getCurrentPosition();
    }
    public void extendIntake() {
        //if()
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();

        setPosition(Position.EXTENDED);

        intakeLift.extractIntakeLift();
    }

    public void retractIntake() {
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.EXTRACT)
            intakeLift.prepareIntakeLift();

        setPosition(Position.DEFAULT);

        intakeLift.retractIntakeLift();
    }
    public Position getPosition() {
        synchronized (positionLock) {
            return currentPosition;
        }
    }
    public void stopThread() {
        killController = true;
        try {
            posThread.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private class PositionPlayer implements Runnable {
        @Override
        public void run() {
            while (!killController) {
                synchronized (positionLock) {
                    if (!intakeMotor.isBusy()) {
                        intakeMotor.setPower(0); // Turn off motor when target is reached
                    }
                }
                try {
                    Thread.sleep(100); // Prevent excessive CPU usage
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

}