package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Odometry {
    private DcMotor leftEncoder, rightEncoder;
    private double robotX, robotY, robotHeading;

    public Odometry(DcMotor leftEncoder, DcMotor rightEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
    }

    public void update() {
        int leftTicks = leftEncoder.getCurrentPosition();
        int rightTicks = rightEncoder.getCurrentPosition();

        double leftDistance = leftTicks * Config.TICKS_TO_CM;
        double rightDistance = rightTicks * Config.TICKS_TO_CM;

        double deltaX = (leftDistance + rightDistance) / 2.0 * Math.cos(robotHeading);
        double deltaY = (leftDistance + rightDistance) / 2.0 * Math.sin(robotHeading);

        robotX += deltaX;
        robotY += deltaY;
        robotHeading = Math.atan2(deltaY, deltaX);
    }

    public Pose2d getPose() {
        return new Pose2d(robotX, robotY, new Rotation2d(robotHeading));
    }
}
