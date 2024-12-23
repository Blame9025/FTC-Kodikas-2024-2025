package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class KodiLocalization {

    HardwareMap hardwareMap;
    Motor verticalEncoder, horizontalEncoder;
    KodiIMU imu;

    IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    Thread updateThread;

    public double x = 0, y = 0, theta = 0;
    public double prevV = 0, prevH = 0;

    public KodiLocalization(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        verticalEncoder = new Motor(hardwareMap, "rightRearMotor");
        horizontalEncoder = new Motor(hardwareMap, "rightFrontMotor");

        imu = new KodiIMU(hardwareMap);
        imu.init();
        imu.reset();
        imu.invertGyro();

        verticalEncoder.setDistancePerPulse(Config.TICKS_TO_CM);
        horizontalEncoder.setDistancePerPulse(Config.TICKS_TO_CM);

        verticalEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
    }

    public void start(){
        updateThread = new Thread(() -> {

            while (!updateThread.isInterrupted()){

                double heading = imu.getHeading();

                theta = (int)(heading + 360) % 360;

                double dV = verticalEncoder.getDistance() - prevV;
                double dH = horizontalEncoder.getDistance() - prevH;

                prevV += dV;
                prevH += dH;

                double robotAngle = Math.toRadians(theta);

                double deltaX = - dH * Math.sin(robotAngle) - dV * Math.cos(robotAngle);
                double deltaY = - dH * Math.cos(robotAngle) + dV * Math.sin(robotAngle);

                x += deltaX;
                y += deltaY;
            }

        });
        updateThread.start();
    }

    public void stop(){
        updateThread.interrupt();
    }

}
