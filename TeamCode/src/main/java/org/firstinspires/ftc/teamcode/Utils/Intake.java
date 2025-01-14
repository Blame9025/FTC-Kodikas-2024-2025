package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

public class Intake {
    private DcMotor motorIntake;
   // private final Object positionLock = new Object();
    private double currentPosition = Position.DEFAULT.val;
    private IntakeLift intakeLift;
    private DcMotor coreHex;
    private Outake outtake;
    private Timing.Timer cooldown = new Timing.Timer(800,TimeUnit.MILLISECONDS);
    private Telemetry telemetry;
    public enum Position {
        DEFAULT(0),
        EXTENDED(500);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    KodikasRobot robot;
    public Intake(KodikasRobot robot,DcMotor motorIntake, DcMotor coreHex) {
        this.motorIntake = motorIntake;
        this.robot = robot;
        this.coreHex = coreHex;
        this.outtake = robot.getOutakeSession();
        this.telemetry = robot.getTelemetry();

        cooldown.start();

        coreHex.setDirection(DcMotor.Direction.REVERSE);
    }



    public void setPosition(Position target,boolean back) {
        if (currentPosition != target.val) {
            motorIntake.setTargetPosition(target.val);
            motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorIntake.setPower(back ? -1 : 1);
            while(motorIntake.isBusy());
            motorIntake.setPower(back ? -0.25 : 0.25);
            currentPosition = target.val;
        }
    }

    public void modifyPosition(double value){
        int newPos = Range.clip((int)(currentPosition + value),
                    Position.DEFAULT.val, Position.EXTENDED.val);
        motorIntake.setTargetPosition(newPos);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(newPos > currentPosition ? 1 : -1 );
        currentPosition = newPos;
    }
    Thread extend;
    public void extendIntake() {
        if(extend != null) if(extend.isAlive()) return;
        this.intakeLift = robot.getIntakeLiftSession();
        this.outtake = robot.getOutakeSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();

        if(outtake.getMotorPosition() == Outake.Position.DEFAULT.val){
            outtake.grabbSpecimen();
        }

        extend = new Thread(() -> {
            setPosition(Position.EXTENDED,false);
            cooldown.start();
            while(!cooldown.done());
            intakeLift.extractIntakeLift();
        });
        extend.start();
    }

    Thread retract;
    public void retractIntake() {
        if(retract != null) if(retract.isAlive()) return;
        this.intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.EXTRACT)
            intakeLift.prepareIntakeLift();

        this.outtake = robot.getOutakeSession();
        if(outtake.getMotorPosition() == Outake.Position.DEFAULT.val){
            outtake.grabbSpecimen();
        }
        retract = new Thread(() -> {
            setPosition(Position.DEFAULT,true);
            while(Math.abs(motorIntake.getCurrentPosition() - Position.DEFAULT.val) > 5) {
                telemetry.addData("Motor encoder position", motorIntake.getCurrentPosition());
                telemetry.update();
            }
            intakeLift.retractIntakeLift();
            stop();
        });
        retract.start();

    }
    public void stop(){
        motorIntake.setPower(0);
    }
    public DcMotor getIntakeMotor(){
        return motorIntake;
    }
//    public void autoPos(){
//        setPosition(Position.AUTO);
//    }

    public DcMotor getCoreHex() {return coreHex; }
}
