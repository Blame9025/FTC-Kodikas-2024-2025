package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outake {

    private DcMotor motorOuttake1,motorOuttake2;
    private Servo servoArm, servArmGrabber;
    private boolean alreadyInActionOuttake = false;
    private int currentPosition = Position.DEFAULT.val;
    KodikasRobot robot;
    double power = 0.8;
    public enum Position {
        DEFAULT(0),
        IDLE(600),
        GRABBSPECIMEN(600),
        SPECIMEN(800), // specimen bar
        BASKET1(1600),
        EXTENDED(2620);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    public Outake(KodikasRobot robot, DcMotor definedOuttakeMotor1, DcMotor definedOuttakeMotor2) {
        this.motorOuttake1 = definedOuttakeMotor1;
        this.motorOuttake2 = definedOuttakeMotor2;
        this.robot = robot;
        motorOuttake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOuttake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorOuttake1.setTargetPosition(Intake.Position.DEFAULT.val);
        motorOuttake2.setTargetPosition(Intake.Position.DEFAULT.val);
    }

    public void setPosition(Position target) { // setare pozitie pentru glisiera outtake
        if (currentPosition != target.val) {

            motorOuttake1.setTargetPosition(target.val);
            motorOuttake2.setTargetPosition(target.val);

            motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorOuttake1.setPower(power);
            motorOuttake2.setPower(power);

            currentPosition = target.val;
        }

    }

    public void extendOuttake(){
        setPosition(Position.EXTENDED);

    }
    public void idleOuttake(){
        setPosition(Position.IDLE);
    }

    public void setPositionForGrabbSpecimen(){ // se ridica pana la human player
        setPosition(Position.GRABBSPECIMEN);
    }

    public void specimenBar(){ // se ridica pana la bara de specimene
        setPosition(Position.SPECIMEN);
    }

    public void specimenBar100posMinus(){ // se ridica pana la bara de specimene
        setPosition(Position.SPECIMEN);
    }

    public void retractOuttake(){
        setPosition(Position.DEFAULT);
    }


    public void modifyPosition(boolean up){
        int newPos = Range.clip(getMotorPosition() + (up ? 100 : -100),
                Position.IDLE.val, Position.EXTENDED.val);
        motorOuttake1.setTargetPosition(newPos);
        motorOuttake2.setTargetPosition(newPos);
        motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake1.setPower(power);
        motorOuttake2.setPower(power);
        currentPosition = newPos;
    }

    public int getMotorPositionOuttake1(){
        return motorOuttake1.getCurrentPosition();
    }

    public int getMotorPositionOuttake2(){
        return motorOuttake2.getCurrentPosition();
    }

    public int getMotorPosition(){
        return currentPosition;
    }

    public int getCurrentPositionOuttake() {
        return currentPosition; // Return the stored position
    }

    public void stopMotorOuttake(){
        motorOuttake1.setPower(0);
        motorOuttake2.setPower(0);

    }
}
