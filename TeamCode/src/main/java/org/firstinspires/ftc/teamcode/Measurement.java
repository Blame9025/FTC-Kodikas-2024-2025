package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;

@TeleOp
public class Measurement extends LinearOpMode {
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2;
    DcMotor coreHexIntake;
    public void initHw(){

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");

        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
       // servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        //servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        //servoIntake1.setDirection(Servo.Direction.REVERSE); // de la stanga la dreapta cum te uiti spre intake
        servoIntake2.setDirection(Servo.Direction.REVERSE);
        servoIntake1.setDirection(Servo.Direction.FORWARD);


        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // coreHexIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        //MotorEx encoderLeft, encoderRight;
        //encoderLeft = new MotorEx(hardwareMap, "leftEncoder");
        //encoderRight = new MotorEx(hardwareMap, "rightEncoder");

    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHw();
        KodikasRobot robot = new KodikasRobot(
            motorIntake,
            servoIntake1,
            servoIntake2,
            motorOutake1,
            motorOutake2
        );
        waitForStart();

        //while (opModeIsActive()) {
        //    telemetry.addData("SERVO 1 POS", servoIntake1.getPosition());
        //    telemetry.addData("SERVO 2 POS", servoIntake2.getPosition());
//
  //          servoIntake1.setPosition(0.2);
    //        servoIntake2.setPosition(0.2);
      //      telemetry.update();
        //}

        Intake intake = robot.getIntakeSession();
        telemetry.addData("Intake pos inainte", intake.getPosition());
        telemetry.update();
        intake.extendIntake();
        telemetry.addData("Intake pos dupa", intake.getPosition());
        telemetry.update();
    }



}