package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import java.util.concurrent.TimeUnit;

public class IntakeLift {
    private Servo servo1, servo2;
    private boolean alreadyInAction = false;
    private Position currentPosition = Position.DEFAULT;
    KodikasRobot robot;
    Timing.Timer delay = new Timing.Timer(1000, TimeUnit.MILLISECONDS);

    public enum Position {
        DEFAULT(0.2),
        UP(0.6),
        EXTRACT(0.9);

        public final double val;

        Position(double val) {
            this.val = val;
        }
    }

    public IntakeLift(KodikasRobot robot, Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
        this.robot = robot;
        servo1.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(Position target) {
        if (alreadyInAction) return;

        alreadyInAction = true;
        servo1.setPosition(target.val);
        servo2.setPosition(target.val);

        delay.start();
        while (!delay.done()) {}

        currentPosition = target; // Update the current position
        alreadyInAction = false;
    }

    public Position getCurrentPosition() {
        return currentPosition; // Return the stored position
    }

    public void retractIntakeLift() {
        setPosition(Position.DEFAULT);
    }

    public void prepareIntakeLift() {
        setPosition(Position.UP);
    }

    public void extractIntakeLift() {
        setPosition(Position.EXTRACT);
    }
}
