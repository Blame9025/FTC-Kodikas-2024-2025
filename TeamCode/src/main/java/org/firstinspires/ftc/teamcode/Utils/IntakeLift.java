package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;
public class IntakeLift {
    private final Servo servo1, servo2;
    private volatile Position targetPosition = Position.DEFAULT;
    private volatile boolean isActive = false;
    private Thread updateThread;


    public enum Position {
        DEFAULT(0.2),
        UP(0.6),
        EXTRACT(0.9);

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
       // this.servo1.setDirection(Servo.Direction.REVERSE); // Adjust if necessary
        startContinuousUpdate();
    }

    public synchronized void setPosition(Position target) {
        targetPosition = target;
        isActive = true; // Ensure continuous updates are active
    }

    public Position getCurrentPosition() {
        return targetPosition;
    }

    private void startContinuousUpdate() {
        updateThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted()) {
                if (isActive) {
                    servo1.setPosition(targetPosition.val);
                    servo2.setPosition(targetPosition.val);
                }
            }
        });
        updateThread.start();
    }

    public void stopContinuousUpdate() {
        if (updateThread != null) {
            updateThread.interrupt();
            try {
                updateThread.join();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
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
