package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Outake {

    private DcMotor motorOuttake1,motorOuttake2;
    private Servo servoArm, servArmGrabber;
    private boolean alreadyInActionOuttake = false;
    private Position currentPosition = Position.DEFAULT;
    KodikasRobot robot;
    double power = 0.5;
    public enum Position {
        DEFAULT(0),
        UPFOROUTTAKE(850),
        SPECIMEN(1400),
        EXTENDED(2700);

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
        if (currentPosition != target) {

            motorOuttake1.setTargetPosition(target.val);
            motorOuttake2.setTargetPosition(target.val);

            motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorOuttake1.setPower(power);
            motorOuttake2.setPower(power);

            currentPosition = target;
        }

    }

    /*public void goToZeroPositionOuttake(){ // X nu o folosim X

        if(motorOuttake1.getCurrentPosition() >= Position.DEFAULT.val + 5 && motorOuttake2.getCurrentPosition() >= Position.DEFAULT.val + 5){

            motorOuttake1.setTargetPosition(Intake.Position.DEFAULT.val);
            motorOuttake2.setTargetPosition(Intake.Position.DEFAULT.val);

            motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorOuttake1.setPower(power);
            motorOuttake2.setPower(power);

            while(motorOuttake1.getCurrentPosition() > Position.DEFAULT.val + 5 && motorOuttake1.getCurrentPosition() > Position.DEFAULT.val + 5){

                telemetry.addData("Outtake Position", motorOuttake1.getCurrentPosition());
                telemetry.update();

            }

            motorOuttake1.setPower(0.0);
            motorOuttake2.setPower(0.0);
            currentPosition = Position.DEFAULT;

        }


    }*/

    public void extendOuttake(){
        if(currentPosition == Position.EXTENDED){
            return;
        }else{
            setPosition(Position.EXTENDED);
        }

    }

    public void retractOuttake(){
        if(currentPosition == Position.DEFAULT){
            return;
        }else{
            setPosition(Position.DEFAULT);
        }
    }

    public void outtakeUpForIntake(){
        if(currentPosition == Position.UPFOROUTTAKE){
            return;
        }else{
            setPosition(Position.UPFOROUTTAKE);
        }
    }

    public int getMotorPositionOuttake1(){
        return motorOuttake1.getCurrentPosition();
    }

    public int getMotorPositionOuttake2(){
        return motorOuttake2.getCurrentPosition();
    }

    public int getMotorPosition(){
        return (getMotorPositionOuttake1() +
                getMotorPositionOuttake2()) / 2;
    }

    public Position getCurrentPositionOuttake() {
        return currentPosition; // Return the stored position
    }

    public void stopMotorOuttake(){
        motorOuttake1.setPower(0);
        motorOuttake2.setPower(0);

    }
}
