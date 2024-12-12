package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MeasurementOuttake extends LinearOpMode {
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2, servoGrabber, servoArmGrabber;
    DcMotor coreHexIntake;
    public void initHw(){

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");

        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara


        motorOutake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        servoIntake1.setDirection(Servo.Direction.FORWARD);
        servoIntake2.setDirection(Servo.Direction.REVERSE);

        motorOutake1.setDirection(DcMotorSimple.Direction.REVERSE);

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
        waitForStart();
        while(opModeIsActive()){
            motorOutake1.setTargetPosition(150);
            motorOutake2.setTargetPosition(150);
            motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOutake1.setPower(0.2);
            motorOutake2.setPower(0.2);

            while(motorOutake1.isBusy() && motorOutake2.isBusy()){
                telemetry.addData("motorIntake1: ", motorOutake1.getCurrentPosition());
                telemetry.addData("motorIntake2: ", motorOutake2.getCurrentPosition());
                telemetry.update();
            }

            motorOutake1.setPower(0);
            motorOutake2.setPower(0);
        }
    }
}
