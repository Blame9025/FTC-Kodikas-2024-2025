package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IntakeLift;

public class Intake {
    private DcMotor intakeMotor;

    private boolean alreadyInAction = false;
    private Position currentPosition = Position.DEFAULT;
    IntakeLift intakeLift;
    KodikasRobot robot;
    public enum Position {
        DEFAULT(0),
        EXTENDED(140);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    public Intake(KodikasRobot robot, DcMotor definedIntakeMotor) {
        this.intakeMotor = definedIntakeMotor;
        this.robot = robot;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setTargetPosition(Position.DEFAULT.val);

    }

    public void setPosition(Position target) {
        if (alreadyInAction) return;

        intakeMotor.setTargetPosition(target.val);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(0.5);
        if(currentPosition == target) return;
        alreadyInAction = true;

        long startTime = System.currentTimeMillis();
        while (intakeMotor.isBusy()) {
            if(intakeMotor.getCurrentPosition() == 70)
                intakeMotor.setPower(0.2);
            if (System.currentTimeMillis() - startTime > 10000) {
                break;
            }
        }
        currentPosition = target;
        intakeMotor.setPower(0);
        alreadyInAction = false;
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
        intakeLift.prepareIntakeLift();

        setPosition(Position.DEFAULT);

        intakeLift.retractIntakeLift();
    }
    public void stop(){
        intakeMotor.setTargetPosition(Position.DEFAULT.val);
        intakeMotor.setPower(0.0);
        currentPosition = Position.DEFAULT;
    }
    public Position getPosition(){
        return currentPosition;
    }
}
