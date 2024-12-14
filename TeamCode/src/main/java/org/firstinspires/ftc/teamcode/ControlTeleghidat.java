package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

import java.util.concurrent.TimeUnit;

@TeleOp
public class ControlTeleghidat extends LinearOpMode {

    DcMotor motorIntake;
    DcMotor motorOutake1;
    DcMotor motorOutake2;
    Servo servoIntake1,servoIntake2, servoGrabber, servoArmGrabber;
    DcMotor coreHexIntake;
    KodikasRobot robot;
    Motor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    MecanumDrive drive;
    GamepadEx driverOp;
    final double CHASSIS_SPEED = 1;

    boolean intakeExtended = false;
    boolean intakeToStart = false;
    boolean grabberOpened = false;

    Timing.Timer debA1;
    Timing.Timer waitIntakeExtend;
    Timing.Timer debX1;
    Timing.Timer debB1;
    Timing.Timer debY;
    public void initHW(){
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
        motorOutake2.setDirection(DcMotor.Direction.REVERSE);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setDirection(DcMotor.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(
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

        frontLeftMotor = new Motor(hardwareMap, "leftFrontMotor");
        backLeftMotor = new Motor(hardwareMap, "leftRearMotor");
        frontRightMotor = new Motor(hardwareMap, "rightFrontMotor");
        backRightMotor = new Motor(hardwareMap, "rightRearMotor");

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);
        driverOp = new GamepadEx(gamepad2);

        debA1 = new Timing.Timer(300,TimeUnit.MILLISECONDS);
        debA1.start();

        debX1 = new Timing.Timer(300,TimeUnit.MILLISECONDS);
        debX1.start();

        debB1 = new Timing.Timer(300,TimeUnit.MILLISECONDS);
        debB1.start();

        debY = new Timing.Timer(300,TimeUnit.MILLISECONDS);
        debY.start();

        waitIntakeExtend = new Timing.Timer(500,TimeUnit.MILLISECONDS);
        waitIntakeExtend.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();
            waitForStart();

            Intake intake = robot.getIntakeSession();
            Outake outake = robot.getOutakeSession();
            OuttakeLift outakeLift = robot.getOutakeLiftsession();
            while(opModeIsActive()){

                drive.driveRobotCentric(
                        -driverOp.getLeftX() * CHASSIS_SPEED,
                        -driverOp.getLeftY() * CHASSIS_SPEED,
                        -driverOp.getRightX() * CHASSIS_SPEED,
                        true
                );
                if(gamepad1.dpad_up) {
                    outake.extendOuttake();
                }
                if(gamepad1.dpad_down){
                    outake.outtakeUpForIntake();
                }
                if(gamepad1.a && debA1.done()){

                    if(!intakeExtended){
                        intake.extendIntake();
                        intakeToStart = true;
                        waitIntakeExtend.start();
                    }
                    else {
                        outake.outtakeUpForIntake();
                        intake.retractIntake();
                    }

                    intakeExtended = !intakeExtended;
                    debA1.start();
                }

                if(intakeToStart && waitIntakeExtend.done()){
                    coreHexIntake.setPower(1);
                    intakeToStart = false;
                }
                if(gamepad1.x && debX1.done()){
                    if(coreHexIntake.getPower() != 1.0) coreHexIntake.setPower(1);
                    else coreHexIntake.setPower(0);
                    debX1.start();
                }
                if(gamepad1.b && debB1.done()){
                    if(coreHexIntake.getPower() != -0.5) coreHexIntake.setPower(-0.5);
                    else coreHexIntake.setPower(0);
                    debB1.start();
                }
                if(gamepad1.y && debY.done())
                {
                    if(!grabberOpened){
                        outakeLift.openGrabber();
                    }
                    else
                        outakeLift.closeGrabber();
                    grabberOpened = !grabberOpened;
                    debY.start();
                }
                if(gamepad1.right_stick_button && intake.getCurrentPosition() < -100){
                    Thread t = new Thread(() -> {
                        outake.retractOuttake();
                        outakeLift.openGrabber();
                        outakeLift.downArmGrabber();
                        Timing.Timer timer = new Timing.Timer(800,TimeUnit.MILLISECONDS);
                        timer.start();
                        while (!timer.done());
                        outakeLift.closeGrabber();
                        outake.extendOuttake();
                        timer.start();
                        while (!timer.done());
                        outakeLift.upArmGrabber();
                        grabberOpened = false;
                    });
                }
                if(gamepad1.right_bumper)
                    outakeLift.downArmGrabber();

                if(gamepad1.left_bumper)
                    outakeLift.upArmGrabber();
                if(Math.abs(gamepad1.right_trigger - gamepad1.left_trigger) > 0.01){
                    intake.modifyPosition(gamepad1.right_trigger - gamepad1.left_trigger);
                }
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }
    }
}
