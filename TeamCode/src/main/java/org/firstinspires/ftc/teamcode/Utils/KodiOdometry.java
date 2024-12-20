package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class KodiOdometry {
    //MotorEx verticalEncoder, horizontalEncoder;
    HolonomicOdometry holoOdometry;
   // OdometrySubsystem odometry;
    IMU imu;

    double offset;
    public KodiOdometry(IMU imu, Motor verticalEncoder, Motor horizontalEncoder) {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        verticalEncoder.setDistancePerPulse(Config.TICKS_TO_CM);
        horizontalEncoder.setDistancePerPulse(Config.TICKS_TO_CM);
        verticalEncoder.resetEncoder();
        horizontalEncoder.resetEncoder();
        holoOdometry = new HolonomicOdometry(
                () -> verticalEncoder.getDistance() +
                        imu.getRobotOrientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.RADIANS
                        ).firstAngle,
                () -> verticalEncoder.getDistance() -
                        imu.getRobotOrientation(
                                AxesReference.INTRINSIC,
                                AxesOrder.ZYX,
                                AngleUnit.RADIANS
                        ).firstAngle,
                horizontalEncoder::getDistance,
                Config.TRACKWIDTH, Config.CENTER_WHEEL_OFFSET
        );
    }
    public HolonomicOdometry getHolonomicOdometry() {
        return this.holoOdometry;
    }
    public static GeneralWaypoint wp(double x, double y, double angle){
        return new GeneralWaypoint(
                -x,-y,Math.toRadians(angle),0.8,0.8,30
        );
    }
    public static EndWaypoint ewp(double x, double y, double angle){
        return new EndWaypoint(
                -x,-y,Math.toRadians(angle),0.6,0.5,30,0.8,1
        );
    }
}
