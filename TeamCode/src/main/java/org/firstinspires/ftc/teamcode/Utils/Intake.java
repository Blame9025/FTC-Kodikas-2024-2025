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

        if(intakeMotor.isBusy()) return;
        intakeMotor.setTargetPosition(target.val);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        boolean isHalfWayThere = intakeMotor.getCurrentPosition() > Position.EXTENDED.val / 2;
        intakeMotor.setPower(isHalfWayThere ? 0.2 : 0.5);
        if(currentPosition == target) return;
        currentPosition = target;

    }
    public int getMotorPosition(){
        return intakeMotor.getCurrentPosition();
    }
    public void extendIntake() {
        //if()
        if(currentPosition == Position.EXTENDED) return;
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();

        setPosition(Position.EXTENDED);

        intakeLift.extractIntakeLift();
    }

    public void retractIntake() {
        if(currentPosition == Position.DEFAULT) return;
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.EXTRACT)
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
