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
    private final DcMotor intakeMotor;
    private int currentPosition = Position.DEFAULT.val;
   // private final Object positionLock = new Object();
    private IntakeLift intakeLift;
    private DcMotor coreHex;
    private Outake outtake;
    public enum Position {
        DEFAULT(-50),
        EXTENDED(900);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }
    KodikasRobot robot;
    public Intake(KodikasRobot robot,DcMotor intakeMotor, DcMotor coreHex) {
        this.intakeMotor = intakeMotor;
        this.robot = robot;
        this.coreHex = coreHex;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        coreHex.setDirection(DcMotor.Direction.REVERSE);
    }



    public void setPosition(Position target) {
        if (currentPosition != target.val) {
            intakeMotor.setTargetPosition(target.val);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(1.0);
            currentPosition = target.val;
        }
    }

    public void modifyPosition(double value){
        int newPos = Range.clip(intakeMotor.getCurrentPosition() + (int)(value*100),
                Position.DEFAULT.val, Position.EXTENDED.val);
        intakeMotor.setTargetPosition(newPos);
        intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakeMotor.setPower(0.8);
        currentPosition = newPos;
    }
    Thread extend;
    public void extendIntake() {
        if(extend != null) if(extend.isAlive()) return;
        this.intakeLift = robot.getIntakeLiftSession();
        this.outtake = robot.getOutakeSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();
        if(outtake.getCurrentPositionOuttake() == Outake.Position.DEFAULT)
            outtake.outtakeUpForIntake();
        extend = new Thread(() -> {
            Timing.Timer timer = new Timing.Timer(250,TimeUnit.MILLISECONDS);
            timer.start();
            while (!timer.done());
            setPosition(Position.EXTENDED);
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
        retract = new Thread(() -> {
            Timing.Timer timer = new Timing.Timer(200,TimeUnit.MILLISECONDS);
            timer.start();
            while (!timer.done());
            setPosition(Position.DEFAULT);
            timer = new Timing.Timer(800,TimeUnit.MILLISECONDS);
            timer.start();
            while (!timer.done());
            intakeLift.retractIntakeLift();
            stop();
        });
        retract.start();

    }
    public void stop(){
        intakeMotor.setPower(0);
    }

    public DcMotor getCoreHex() {return coreHex; }
    public int getCurrentPosition() {
        return currentPosition;
    }
    public DcMotor getIntakeMotor(){ return intakeMotor; }
}
