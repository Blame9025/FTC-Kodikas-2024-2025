package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
public class Intake {
    private final DcMotor intakeMotor;
    private Position currentPosition = Position.DEFAULT;
   // private final Object positionLock = new Object();
    private IntakeLift intakeLift;

    public enum Position {
        DEFAULT(-350),
        EXTENDED(500);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }
    KodikasRobot robot;
    public Intake(KodikasRobot robot,DcMotor intakeMotor) {
        this.intakeMotor = intakeMotor;
        this.robot = robot;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setPosition(Position target) {
        if (currentPosition != target) {
            intakeMotor.setTargetPosition(target.val);
            intakeMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(1.0);
            currentPosition = target;
        }
    }

    public void extendIntake() {
        this.intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();
        setPosition(Position.EXTENDED);
        intakeLift.extractIntakeLift();
    }

    public void retractIntake() {
        this.intakeLift = robot.getIntakeLiftSession();
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.EXTRACT)
            intakeLift.prepareIntakeLift();
        setPosition(Position.DEFAULT);

    }
    public void stop(){
        intakeMotor.setPower(0);
    }
    public Position getCurrentPosition() {
        return currentPosition;
    }
    public DcMotor getIntakeMotor(){ return intakeMotor; }
}
