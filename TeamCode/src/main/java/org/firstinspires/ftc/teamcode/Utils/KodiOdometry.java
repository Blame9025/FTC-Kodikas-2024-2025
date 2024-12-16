package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class KodiOdometry {
    MotorEx verticalEncoder, horizontalEncoder;
    HolonomicOdometry holoOdometry;
    OdometrySubsystem odometry;
    IMU imu;
    public KodiOdometry(IMU imu, Motor verticalEncoder, Motor horizontalEncoder) {
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        holoOdometry = new HolonomicOdometry(
                () -> verticalEncoder.getCurrentPosition() * Config.TICKS_TO_CM +
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                () -> verticalEncoder.getCurrentPosition() * Config.TICKS_TO_CM -
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
                () -> -horizontalEncoder.getCurrentPosition() * Config.TICKS_TO_CM,
                Config.TRACKWIDTH, Config.CENTER_WHEEL_OFFSET
        );
    }
    public HolonomicOdometry getHolonomicOdometry() {
        return this.holoOdometry;
    }

}
