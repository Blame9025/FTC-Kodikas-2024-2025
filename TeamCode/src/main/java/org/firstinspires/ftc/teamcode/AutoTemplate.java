package org.firstinspires.ftc.teamcode;

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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.*;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class AutoTemplate extends LinearOpMode {
    Motor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor leftEncoder, rightEncoder;
    DcMotor intakeMotor;
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2;
    DcMotor coreHexIntake;
    Servo servoGrabber, servoArmGrabber;

    void initHw()
    {

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");


        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara


    }


    @Override
    public void runOpMode() throws InterruptedException {
        initHw();
        KodikasRobot robot = new KodikasRobot(
            telemetry,
            motorIntake,
            coreHexIntake,
            servoIntake1,
            servoIntake2,
            motorOutake1,
            motorOutake2,
            servoGrabber,
            servoArmGrabber
        );
        MecanumDrive drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        GamepadEx driverOp = new GamepadEx(gamepad1);
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        KodiOdometry kodiOdometry = new KodiOdometry(imu, leftEncoder, rightEncoder);
        OdometrySubsystem odometry = new OdometrySubsystem(kodiOdometry.getHolonomicOdometry());
        waitForStart();

        PurePursuitCommand PureppCmd = new PurePursuitCommand(
                drive, odometry,
                new StartWaypoint(odometry.getPose()),
                new GeneralWaypoint(
                        90,-46, Math.toRadians(75),
                        0.6, 0.5, 30)

        );

    }
}
