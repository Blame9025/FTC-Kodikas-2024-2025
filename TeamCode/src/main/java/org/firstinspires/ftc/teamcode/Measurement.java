package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;

import java.util.concurrent.TimeUnit;

@TeleOp
public class Measurement extends LinearOpMode {
    DcMotor motorIntake;
    DcMotor motorGlisiera;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2, servoGrabber, servoArmGrabber;
    DcMotor coreHexIntake;
    Timing.Timer delay = new Timing.Timer(3000, TimeUnit.MILLISECONDS);
    public void initHw(){

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        servoIntake1.setDirection(Servo.Direction.FORWARD);
        servoIntake2.setDirection(Servo.Direction.REVERSE);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setDirection(DcMotor.Direction.REVERSE);
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        waitForStart();
        Intake intake = robot.getIntakeSession();
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        try{
            intake.extendIntake();
            while(intake.getIntakeMotor().isBusy() && opModeIsActive()){
                telemetry.addData("INTAKE MOTOR POS",intake.getIntakeMotor().getCurrentPosition());
                telemetry.update();
            }
            intake.stop();
            telemetry.addData("INTAKE MOTOR STOPPED", intake.getIntakeMotor().getPower());
            telemetry.update();
            long startTime = System.currentTimeMillis();
            while(System.currentTimeMillis() - startTime < 2500 && opModeIsActive())
            {
                coreHexIntake.setPower(1);
            }
            telemetry.addData("Retracting from position",intake.getIntakeMotor().getCurrentPosition());
            telemetry.update();
            intake.retractIntake();
            sleep(500 );
            while(intake.getIntakeMotor().isBusy() && opModeIsActive()){
                telemetry.addData("INTAKE MOTOR POS 2",intake.getIntakeMotor().getCurrentPosition());
                telemetry.update();
            }
            telemetry.addData("Retracted",true);
            telemetry.update();
            intake.stop();
            sleep(50);
            intakeLift.retractIntakeLift();
            sleep(200);
            delay.start();
            while( !delay.done() && opModeIsActive())
            {
                coreHexIntake.setPower(-1.0);
            }
            coreHexIntake.setPower(0);
            throw new InterruptedException();
        }catch(InterruptedException e){
            intake.getIntakeMotor().setTargetPosition(0);
            intake.stop();
            intakeLift.stopContinuousUpdate();
            telemetry.addData("OPRIM TOT", true);
            telemetry.update();
        }


    }



}