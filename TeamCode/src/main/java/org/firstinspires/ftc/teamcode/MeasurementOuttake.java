package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class MeasurementOuttake extends LinearOpMode {
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2, servoGrabber, servoArmGrabber;
    DcMotor coreHexIntake;
    Timing.Timer delay = new Timing.Timer(5000, TimeUnit.MILLISECONDS);
    Timing.Timer active = new Timing.Timer(2000, TimeUnit.MILLISECONDS);
    final int positionUp = 600;
    final int postionBasket = 1200;
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
        motorOutake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        servoIntake1.setDirection(Servo.Direction.FORWARD);
        servoIntake2.setDirection(Servo.Direction.REVERSE);

        motorOutake1.setDirection(DcMotorSimple.Direction.FORWARD);
        motorOutake2.setDirection(DcMotorSimple.Direction.REVERSE);

        //servoArmGrabber.setDirection(Servo.Direction.REVERSE);

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

            /*telemetry.addData("motorIntake1: ", motorOutake1.getCurrentPosition());
            telemetry.addData("motorIntake2: ", motorOutake2.getCurrentPosition());
            telemetry.update();*/


            /*telemetry.addData("servoGrabber: ", servoGrabber.getPosition());
            telemetry.update();*/

            //CALIBRARE ARM GRABBER START

            /*servoArmGrabber.setPosition(0);
            delay.start();*/


            //CALIBRARE ARM GRABBER END

            //START SECTION MEASUREMENT FOR MOTOR

            /*motorOutake1.setTargetPosition(600);
            motorOutake2.setTargetPosition(600);
            motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorOutake1.setPower(0.5);
            motorOutake2.setPower(0.5);

            while(motorOutake1.isBusy() && motorOutake2.isBusy()){
                telemetry.addData("motorIntake1: ", motorOutake1.getCurrentPosition());
                telemetry.addData("motorIntake2: ", motorOutake2.getCurrentPosition());
                telemetry.update();
            }

            motorOutake1.setPower(0);
            motorOutake2.setPower(0);

            /*active.start();
            while(!active.done()) {
                motorOutake1.setPower(0.5);
                motorOutake2.setPower(0.5);
            }
            motorOutake1.setPower(0);
            motorOutake2.setPower(0);*/

            //END SECTION MEASUREMENT FOR MOTOR

            //START SECTION MEASUREMENT FOR GRABBER

            servoGrabber.setPosition(0.1);

            delay.start();
            while(!delay.done()){
                telemetry.addData("ServoGrabber: ", servoGrabber.getPosition());
                telemetry.update();
            }

            servoGrabber.setPosition(0);

            delay.start();
            while(!delay.done()){
                telemetry.addData("ServoGrabber: ", servoGrabber.getPosition());
                telemetry.update();
            }

            servoGrabber.setPosition(0);

            //END SECTION MEASUREMENT FOR GRABBER


            //START SECTION MEASUREMENT FOR ARM GRABBER

            /*servoArmGrabber.setPosition(0);

            delay.start();
            while(!delay.done() && opModeIsActive()){
                telemetry.addData("ServoGrabber: ", servoGrabber.getPosition());
                telemetry.update();
            }

            servoArmGrabber.setPosition(0.4);

            delay.start();
            while(!delay.done() && opModeIsActive()){
                telemetry.addData("ServoGrabber: ", servoGrabber.getPosition());
                telemetry.update();
            }

            servoArmGrabber.setPosition(0);*/


            //END SECTION MEASUREMENT FOR ARM GRABBER

        }
    }
}
