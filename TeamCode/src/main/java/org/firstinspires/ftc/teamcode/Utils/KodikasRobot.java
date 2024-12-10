package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class KodikasRobot {
    Intake intake;
    IntakeLift intakeLift;
    Outake outake;

    public KodikasRobot(DcMotor motorIntake, Servo servo1, Servo servo2, DcMotor outakeMotor1, DcMotor outakeMotor2){
        this.intake = new Intake(motorIntake);
        this.intakeLift = new IntakeLift(servo1, servo2);
        this.outake = new Outake(outakeMotor1, outakeMotor2);
    }
    public Intake getIntakeSession(){
        return intake;
    }
    public IntakeLift getIntakeLiftSession(){
        return intakeLift;
    }
    public Outake getOutakeSession(){
        return outake;
    }
}
