package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class IntakeLift {
    private Servo servo1, servo2;
    private boolean alreadyInAction = false;
    private Position currentPosition = Position.DEFAULT;

    Timing.Timer delay = new Timing.Timer(1000, TimeUnit.MILLISECONDS);

    public enum Position {
        DEFAULT(0),
        UP(0.5),
        EXTRACT(1);

        public final double val;

        Position(double val) {
            this.val = val;
        }
    }

    public void IntakeLiftController(Servo servo1, Servo servo2) {
        this.servo1 = servo1;
        this.servo2 = servo2;
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
