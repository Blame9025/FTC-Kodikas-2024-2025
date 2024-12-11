package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        initHw();
        KodikasRobot robot = new KodikasRobot(
            motorIntake,
            servoIntake1,
            servoIntake2,
            motorOutake1,
            motorOutake2,
            servoGrabber,
            servoArmGrabber
        );
        waitForStart();
        Intake intake = robot.getIntakeSession();
        long startTime = System.currentTimeMillis();
        boolean retract = false;
        while (opModeIsActive()) {
            if(retract)
                intake.retractIntake();
            else
                intake.extendIntake();
            //intake.extendIntake();
            telemetry.addData("intake motor pos",intake.getMotorPosition());
            telemetry.update();
            if (System.currentTimeMillis() - startTime > 7500 && !retract) {
                retract = true;
            }

        }

    }



}