package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.Servo;

public class IntakeLift {
    private Servo servo1, servo2;
    private volatile Position targetPosition = Position.DEFAULT;
    private Thread positionThread;
    private volatile boolean isRunning = false;
    private KodikasRobot robot;
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
        this.servo1.setDirection(Servo.Direction.REVERSE);
    }

    public synchronized void setPosition(Position target) {
        // Update the target position
        targetPosition = target;

        // If a thread is already running, stop it before starting a new one
        if (isRunning) {
            stopPositionThread();
        }

        // Start a new thread to handle the position update
        startPositionThread();
    }

    private synchronized void startPositionThread() {
        isRunning = true;
        positionThread = new Thread(() -> {
            while (isRunning) {
                // Continuously set the servos to the target position
                servo1.setPosition(targetPosition.val);
                servo2.setPosition(targetPosition.val);

                try {
                    Thread.sleep(50); // Adjust frequency as needed
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt(); // Handle thread interruption
                }
            }
        });
        positionThread.start();
    }

    public synchronized void stopPositionThread() {
        isRunning = false;
        if (positionThread != null) {
            try {
                positionThread.join(); // Wait for the thread to finish
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
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

    public void extractIntakeLift() {
        setPosition(Position.EXTRACT);
    }
}
