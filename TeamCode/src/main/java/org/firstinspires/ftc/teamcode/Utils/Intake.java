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
        EXTENDED(170);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    public Intake(KodikasRobot robot, DcMotor definedIntakeMotor) {
        this.intakeMotor = definedIntakeMotor;
        this.robot = robot;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setTargetPosition(Position.DEFAULT.val);
    }

    public void setPosition(Position target) {
        if (alreadyInAction) return;

        alreadyInAction = true;

        intakeMotor.setTargetPosition(target.val);
        intakeMotor.setPower(0.5);

        long startTime = System.currentTimeMillis();
        while (intakeMotor.isBusy()) {
            if (System.currentTimeMillis() - startTime > 5000) {
                break;
            }
        }
        currentPosition = target;
        intakeMotor.setPower(0);
        alreadyInAction = false;
    }

    public void extendIntake() {
        //if()
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();
        long startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 5000) {
        }
        //setPosition(Position.EXTENDED);

        intakeLift.extractIntakeLift();
        startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < 10000) {
        }
    }

    public void retractIntake() {

        robot.getIntakeLiftSession().prepareIntakeLift();

        setPosition(Position.DEFAULT);

        robot.getIntakeLiftSession().retractIntakeLift();
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
