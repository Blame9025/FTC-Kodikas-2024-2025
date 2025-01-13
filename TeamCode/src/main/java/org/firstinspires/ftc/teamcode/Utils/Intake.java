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
    private int currentPosition = Position.DEFAULT.val;
    private IntakeLift intakeLift;
    private DcMotor coreHex;
    private Outake outtake;
    private Timing.Timer cooldown = new Timing.Timer(800,TimeUnit.MILLISECONDS);

    public enum Position {
        DEFAULT(0),
        EXTENDED(50);

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
        outtake = robot.getOutakeSession();

        cooldown.start();

        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHex.setDirection(DcMotor.Direction.REVERSE);

    }



    public void setPosition(Position target) {
        if (currentPosition != target.val) {
            motorIntake.setTargetPosition(target.val);
            motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorIntake.setPower(1.0);
            currentPosition = target.val;
        }
    }

    public void modifyPosition(int value){
        int newPos = Range.clip(currentPosition + value,
                    Position.DEFAULT.val, Position.EXTENDED.val);
        motorIntake.setTargetPosition(newPos);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(1.0);
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
            setPosition(Position.EXTENDED);
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

        if(outtake.getMotorPosition() == Outake.Position.DEFAULT.val){
            outtake.grabbSpecimen();
        }
        retract = new Thread(() -> {
            Timing.Timer timer = new Timing.Timer(200,TimeUnit.MILLISECONDS);
            timer.start();
            while (!timer.done());
            cooldown.start();
            setPosition(Position.DEFAULT);
            while(!cooldown.done());
            intakeLift.retractIntakeLift();
            stop();
        });
        retract.start();

    }
    public void stop(){
       motorIntake.setPower(0);
    }

//    public void autoPos(){
//        setPosition(Position.AUTO);
//    }

    public DcMotor getCoreHex() {return coreHex; }
}
