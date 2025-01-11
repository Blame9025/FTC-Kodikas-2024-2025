package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

@TeleOp
public class testGhearaSiBrat extends LinearOpMode {
    Servo servoArm;
    Servo servoArmGrabber;

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;


    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap, telemetry);
        drive = robot.getDriveSession();

        dist = new KodiDistance(hardwareMap,drive);

        loc = new KodiLocalization(hardwareMap);
        loc.start();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        initHW();
        servoArm = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara
        //servoArm.setDirection(Servo.Direction.REVERSE);
        servoArm.setDirection(Servo.Direction.REVERSE);



        waitForStart();
        double pos = 0;
        servoArm.setPosition(0.1);
        sleep(3000);
        servoArm.setPosition(0);
        //servoArm.setPosition(0);
        while(opModeIsActive()){

        }
    }
}

































































