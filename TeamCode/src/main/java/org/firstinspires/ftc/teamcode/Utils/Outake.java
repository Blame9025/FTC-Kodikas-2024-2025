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
        UPFORINTAKE(50),
        EXTENDED(250);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    public Outake(KodikasRobot robot, DcMotor definedOuttakeMotor1, DcMotor definedOuttakeMotor2, Servo servoArmGrabber, Servo servoArm) {
        this.motorOuttake1 = definedOuttakeMotor1;
        this.motorOuttake2 = definedOuttakeMotor2;
        this.servoArm = servoArm;
        this.servArmGrabber = servoArmGrabber;
        this.robot = robot;
        motorOuttake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOuttake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorOuttake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorOuttake1.setTargetPosition(Intake.Position.DEFAULT.val);
        motorOuttake2.setTargetPosition(Intake.Position.DEFAULT.val);
    }

    public void setPosition(Position target) { // setare pozitie pentru glisiera outtake

        if(motorOuttake1.isBusy()) return;
        if(motorOuttake2.isBusy()) return;
        if(alreadyInActionOuttake){
            return;
        }
        alreadyInActionOuttake = true;

        motorOuttake1.setTargetPosition(target.val);
        motorOuttake2.setTargetPosition(target.val);

        motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorOuttake1.setPower(power);
        motorOuttake2.setPower(power);

        long startTime = System.currentTimeMillis();
        while(motorOuttake1.isBusy() && motorOuttake2.isBusy()){
            if(System.currentTimeMillis() - startTime > 7000){
                break;
            }
        }

        while(motorOuttake1.isBusy() && motorOuttake2.isBusy()){
            telemetry.addData("Outtake Position", motorOuttake1.getCurrentPosition());
            telemetry.update();

        }

        if(motorOuttake1.getCurrentPosition() >= target - 5 && motorOuttake2.getCurrentPosition() >= target - 5){
            motorOuttake1.setPower(0);
            motorOuttake2.setPower(0);

        }

        currentPosition = target;

        alreadyInActionOuttake = false;

    }

    public void goToZeroPositionOuttake(){

        if(motorOuttake1.getCurrentPosition() >= Position.DEFAULT + 5 && motorOuttake2.getCurrentPosition() >= Position.DEFAULT + 5){

            motorOuttake1.setTargetPosition(Intake.Position.DEFAULT.val);
            motorOuttake2.setTargetPosition(Intake.Position.DEFAULT.val);

            motorOuttake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOuttake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorOuttake1.setPower(power);
            motorOuttake2.setPower(power);

            while(motorOuttake1.getCurrentPosition() > Position.DEFAULT + 5 && motorOuttake1.getCurrentPosition() > Position.DEFAULT + 5){

                telemetry.addData("Outtake Position", motorOuttake1.getCurrentPosition());
                telemetry.update();

            }

            motorOuttake1.setPower(0.0);
            motorOuttake2.setPower(0.0);
            currentPosition = Position.DEFAULT;

        }


    }

    public int getMotorPositionOuttake1(){
        return motorOuttake1.getCurrentPosition();
    }

    public int getMotorPositionOuttake2(){
        return motorOuttake2.getCurrentPosition();
    }

    public Position getCurrentPositionOuttake() {
        return currentPosition; // Return the stored position
    }
}
