package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PWMOutput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outake {

    private DcMotor motorOuttake1,motorOuttake2;
    private int currentPosition = Position.DEFAULT.val;
    KodikasRobot robot;
    double power = 1;
    public enum Position {
        DEFAULT(0),
        SPECIMENAUTO(500),
        IDLE(600),
        GRABBSPECIMEN(500),
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

    public void grabbSpecimen(){ // se ridica pana la human player
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

    public void autoSpec2(){setPosition(Position.SPECIMENAUTO);}


    public void modifyPosition(boolean up, boolean forced){
        int newPos = Range.clip(getMotorPosition() + (up ? 100 : -100),
                forced ? -1000 : Position.IDLE.val, Position.EXTENDED.val);
        motorOuttake1.setTargetPosition(newPos);
        motorOuttake2.setTargetPosition(newPos);
        motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake1.setPower(power);
        motorOuttake2.setPower(power);
        currentPosition = newPos;
    }

    public boolean isBusy(){
        return motorOuttake1.isBusy() && motorOuttake2.isBusy();
    }

    public DcMotor getMotorOuttake2(){
        return motorOuttake2;
    }

    public DcMotor getMotorOuttake1(){

        return motorOuttake2;
    }

    public int getMotorPosition(){
        return currentPosition;
    }

    public void stopMotorOuttake(){
        motorOuttake1.setPower(0);
        motorOuttake2.setPower(0);

    }
}
