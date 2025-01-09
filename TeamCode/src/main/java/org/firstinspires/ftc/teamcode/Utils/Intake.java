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
    private Servo servoIntake1,servoIntake2;
   // private final Object positionLock = new Object();
    private double currentPosition = Position.DEFAULT.val;
    private IntakeLift intakeLift;
    private DcMotor coreHex;

    public enum Position {
        DEFAULT(0.0),
        EXTENDED(1.0);

        public final double val;

        Position(double val) {
            this.val = val;
        }
    }

    KodikasRobot robot;
    public Intake(KodikasRobot robot,Servo servoIntake1, Servo servoIntake2, DcMotor coreHex) {
        this.servoIntake1 = servoIntake1;
        this.servoIntake2 = servoIntake2;
        this.robot = robot;
        this.coreHex = coreHex;
        coreHex.setDirection(DcMotor.Direction.REVERSE);
        servoIntake2.setDirection(Servo.Direction.REVERSE);
    }



    public void setPosition(Position target) {
        if (currentPosition != target.val) {
            servoIntake2.setPosition(target.val);
            servoIntake1.setPosition(target.val);
            currentPosition = target.val;
        }
    }

    public void modifyPosition(double value){
        double newPos = Range.clip(currentPosition + value,
                    (double)Position.DEFAULT.val, (double)Position.EXTENDED.val);
        servoIntake1.setPosition(newPos);
        servoIntake2.setPosition(newPos);
        currentPosition = newPos;
    }
    Thread extend;
    public void extendIntake() {
        if(extend != null) if(extend.isAlive()) return;
        this.intakeLift = robot.getIntakeLiftSession();
        this.outtake = robot.getOutakeSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();

        extend = new Thread(() -> {
            setPosition(Position.EXTENDED);
            while(intakeMotor.getCurrentPosition() < Position.EXTENDED.val - 50);
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
            while(intakeMotor.getCurrentPosition() > Position.DEFAULT.val + 50 );
            intakeLift.retractIntakeLift();
            stop();
        });
        retract.start();

    }
    public void stop(){
        intakeMotor.setPower(0);
    }

    public void autoPos(){
        setPosition(Position.AUTO);
    }

    public DcMotor getCoreHex() {return coreHex; }
    public int getCurrentPosition() {
        return currentPosition;
    }
    public DcMotor getIntakeMotor(){ return intakeMotor; }
}
