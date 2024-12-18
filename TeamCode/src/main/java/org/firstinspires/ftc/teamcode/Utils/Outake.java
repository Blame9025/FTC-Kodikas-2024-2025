package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Outake {

    private DcMotor motorOuttake1,motorOuttake2;
    private Servo servoArm, servArmGrabber;
    private boolean alreadyInActionOuttake = false;
    private int currentPosition = Position.DEFAULT.val;
    KodikasRobot robot;
    double power = 1;
    public enum Position {
        DEFAULT(0),
        GRABBSPECIMEN(600),
        UPFOROUTTAKE(850),
        SPECIMEN(1400),
        BASKET1(1600),
        EXTENDED(2625);

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
        setPosition(Position.EXTENDED);


    }

    public void retractOuttake(){
        setPosition(Position.DEFAULT);
    }

    public void outtakeUpForIntake(){
        //setPosition(Position.UPFOROUTTAKE);
    }

    public void modifyPosition(boolean up){
        int newPos = Range.clip(getMotorPosition() + (up? 300 : -300),
                Intake.Position.DEFAULT.val, Intake.Position.EXTENDED.val);
        motorOuttake1.setTargetPosition(newPos);
        motorOuttake2.setTargetPosition(newPos);
        motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake1.setPower(0.5);
        motorOuttake2.setPower(0.5);
        currentPosition = newPos;
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

    public int getCurrentPositionOuttake() {
        return currentPosition; // Return the stored position
    }

    public void stopMotorOuttake(){
        motorOuttake1.setPower(0);
        motorOuttake2.setPower(0);

    }
}
