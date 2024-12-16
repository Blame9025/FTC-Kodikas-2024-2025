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

    final int DEBUG_TIMER = 300;

    DcMotor coreHexIntake;
    KodikasRobot robot;
    MecanumDrive drive;
    GamepadEx driverOp;


    boolean intakeExtended = false;
    boolean intakeToStart = false;
    boolean grabberOpened = false;

    Timing.Timer debA1;
    Timing.Timer waitIntakeExtend;
    Timing.Timer debX1;
    Timing.Timer debB1;
    Timing.Timer debY;
    Timing.Timer debStickR1;
    public void initHW(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap,telemetry);
        coreHexIntake = robot.getCoreHexIntake();


        drive = robot.getDriveSession();
        driverOp = new GamepadEx(gamepad2);

        debA1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debA1.start();

        debX1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debX1.start();

        debB1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debB1.start();

        debY = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debY.start();

        debStickR1 = new Timing.Timer(1000,TimeUnit.MILLISECONDS);
        debStickR1.start();

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

            outakeLift.closeGrabber();
            Timing.Timer clawClose = new Timing.Timer(450,TimeUnit.MILLISECONDS);
            clawClose.start();
            while (!clawClose.done());
            outake.outtakeUpForIntake();
            while(opModeIsActive()){

                drive.driveRobotCentric(
                        -driverOp.getLeftX() * (outake.getMotorPosition() > 900 ? 0.5 : 1),
                        -driverOp.getLeftY() * (outake.getMotorPosition() > 900 ? 0.5 : 1),
                        -driverOp.getRightX() * (outake.getMotorPosition() > 900 ? 0.5 : 1),
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
                if(gamepad1.x){
                    coreHexIntake.setPower(1);
                }
                if(gamepad1.b){
                    coreHexIntake.setPower(-0.75);
                }
                if(!gamepad1.b && !gamepad1.x) coreHexIntake.setPower(0);
                if(gamepad1.y && debY.done())
                {
                    if(!grabberOpened)
                        outakeLift.openGrabber();
                    else
                        outakeLift.closeGrabber();
                    grabberOpened = !grabberOpened;
                    debY.start();
                }
                if(gamepad1.right_stick_button && debStickR1.done()){
                    debStickR1.start();
                    Thread t = new Thread(() -> {
                        outake.retractOuttake();
                        outakeLift.openGrabber();
                        outakeLift.downArmGrabber();
                        Timing.Timer timer = new Timing.Timer(1200,TimeUnit.MILLISECONDS);
                        timer.start();
                        while (!timer.done());
                        outakeLift.closeGrabber();

                        timer.start();
                        while (!timer.done());
                        outake.extendOuttake();
                        Timing.Timer timer2 = new Timing.Timer(800,TimeUnit.MILLISECONDS);
                        timer2.start();
                        while(!timer2.done());
                        outakeLift.upArmGrabber();
                        grabberOpened = false;
                    });
                    t.start();
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
