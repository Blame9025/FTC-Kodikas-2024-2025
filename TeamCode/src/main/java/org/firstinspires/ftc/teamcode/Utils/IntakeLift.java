package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;
public class IntakeLift {
    private final Servo servo1, servo2;
    private Position targetPosition = Position.DEFAULT;

    public enum Position {
        DEFAULT(0.9),
        UP(0.6),
        EXTRACT(0.14),
        EXTRACTAUTO(0.15),
        AUTO(1);

        public final double val;

        Position(double val) {
            this.val = val;
        }
    }
    KodikasRobot robot;
    public IntakeLift(KodikasRobot robot,Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.robot = robot;
      //  this.servo2.setDirection(Servo.Direction.REVERSE); // Adjust if necessary
        //startContinuousUpdate();
    }

    public void setPosition(Position target) {
        servo1.setPosition(target.val);
        servo2.setPosition(target.val);
        targetPosition = target;
    }

    public Position getCurrentPosition() {
        return targetPosition;
    }

    public void retractIntakeLift() {
        setPosition(Position.DEFAULT);
    }

    public void prepareIntakeLift() {
        setPosition(Position.UP);
    }

    public void extractIntakeLift() {setPosition(Position.EXTRACT);}

    public void extractIntakeLiftAuto() {setPosition(Position.EXTRACTAUTO);}

    public void autoServoIntake(){setPosition(Position.AUTO);}
}
