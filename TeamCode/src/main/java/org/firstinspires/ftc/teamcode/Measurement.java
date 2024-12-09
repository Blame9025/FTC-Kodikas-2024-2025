package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@TeleOp
public class Measurement extends LinearOpMode {

    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2;
    DcMotor coreHexIntake;
    public void initHw(){
        // Inițializarea hardware-ului
        // Inițializarea hardware-ului pentru motoare
        frontLeftMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        backLeftMotor = hardwareMap.dcMotor.get("leftRearMotor");
        frontRightMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        backRightMotor = hardwareMap.dcMotor.get("rightRearMotor");


        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");

        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
       // servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        //servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servoIntake1.setDirection(Servo.Direction.REVERSE); // de la stanga la dreapta cum te uiti spre intake


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

        // Resetează encoderul înainte de a începe
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configurează motorul pe RUN_TO_POSITION
        motorIntake.setTargetPosition(0);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(0);

        waitForStart();

        int i = 0;
        int max = 170;
        //motorIntake.setPower(0.); // Aplicăm puterea
        motorIntake.setTargetPosition(max);
        int pos = motorIntake.getCurrentPosition();
        while (opModeIsActive() && pos < max) {
            pos = motorIntake.getCurrentPosition();

            motorIntake.setPower(1.0 - pos/max);
            telemetry.addData("Current Position", pos);
            telemetry.addData("Power", motorIntake.getPower());
            telemetry.update();
        }

        motorIntake.setPower(0);
    }



}