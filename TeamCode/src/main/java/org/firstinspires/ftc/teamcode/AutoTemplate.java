package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.*;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class AutoTemplate extends LinearOpMode {
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    DcMotor leftEncoder, rightEncoder;
    DcMotor intakeMotor;
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2;
    DcMotor coreHexIntake;

    void initMotors(){
        frontLeftMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        backLeftMotor = hardwareMap.dcMotor.get("leftRearMotor");
        frontRightMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        backRightMotor = hardwareMap.dcMotor.get("rightRearMotor");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    }
    void initHw()
    {
        initMotors();

        motorGlisiera = hardwareMap.dcMotor.get("motorGlisiera");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");
        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

    }


    @Override
    public void runOpMode() throws InterruptedException {
        initHw();
        Pursuit pursuit = new Pursuit(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor, leftEncoder, rightEncoder);
        List<Waypoint> path = Arrays.asList(
            new Waypoint(new Pose2d(0, 0, new Rotation2d(0)), null),
            new Waypoint(new Pose2d(5, 0, new Rotation2d(0)), () -> {
                telemetry.addData("Wapoint", "1");
                telemetry.update();
            }),
            new Waypoint(new Pose2d(0, 5, new Rotation2d(Math.toRadians(45))), () -> {
                telemetry.addData("Wapoint", "2");
                telemetry.update();
            })
        );
        pursuit.followPathWithActions(path);

    }
}
