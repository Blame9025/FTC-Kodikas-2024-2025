package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Utils.*;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class AutoTemplate extends LinearOpMode {
    DcMotor leftEncoder, rightEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        KodikasRobot robot = new KodikasRobot(
            hardwareMap, telemetry
        );
        MecanumDrive drive = robot.getDriveSession();
        Motor backRightMotor, frontRightMotor;

        backRightMotor = robot.getBRightMotor();
        frontRightMotor = robot.getFRightMotor();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        KodiOdometry kodiOdometry = new KodiOdometry(imu, backRightMotor, frontRightMotor);
        OdometrySubsystem odometry = new OdometrySubsystem(kodiOdometry.getHolonomicOdometry());
        waitForStart();

        while ( opModeIsActive()) {
            //if (isStopRequested()) throw new InterruptedException();

            telemetry.addData("PosX",odometry.getPose().getX());
            telemetry.addData("PosY",odometry.getPose().getY());
            telemetry.addData("Ang",imu.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    AxesOrder.ZXY,
                    AngleUnit.DEGREES
            ).firstAngle);
            odometry.update();

            telemetry.update();
        }

    }
}
