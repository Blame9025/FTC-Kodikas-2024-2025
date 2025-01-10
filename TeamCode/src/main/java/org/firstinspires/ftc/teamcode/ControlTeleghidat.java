package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.Config;
import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiOdometry;
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
    double lastTime;

    boolean intakeExtended = false;
    boolean intakeToStart = false;
    boolean grabberOpened = false;
    boolean specimen = false;
    IMU imu;
    Timing.Timer debA1;
    Timing.Timer waitIntakeExtend;
    Timing.Timer debX1;
    Timing.Timer debB1;
    Timing.Timer debY;
    Timing.Timer debStickR1;
    Timing.Timer debRB1;
    Timing.Timer debMovA;
    Thread shortcutRST;
    Thread y;
    Thread opt;

    DistanceSensor distanceSensor1, distanceSensor2;
    public void initHW(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap,telemetry);
        coreHexIntake = robot.getCoreHexIntake();



        drive = robot.getDriveSession();
        driverOp = new GamepadEx(gamepad2);

        debA1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debA1.start();

        debRB1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debRB1.start();

        debX1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debX1.start();

        debB1 = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debB1.start();

        debY = new Timing.Timer(DEBUG_TIMER,TimeUnit.MILLISECONDS);
        debY.start();

        debStickR1 = new Timing.Timer(1000,TimeUnit.MILLISECONDS);
        debStickR1.start();

        debMovA = new Timing.Timer(5000,TimeUnit.MILLISECONDS);
        debMovA.start();


        waitIntakeExtend = new Timing.Timer(500,TimeUnit.MILLISECONDS);
        waitIntakeExtend.start();

        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distance2");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distance2");
        lastTime = getTime();
    }
    public double getTime(){
        return (double)System.currentTimeMillis() * 0.001;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();
            waitForStart();

            Intake intake = robot.getIntakeSession();
            IntakeLift intakeLift = robot.getIntakeLiftSession();
            Outake outake = robot.getOutakeSession();
            OuttakeLift outakeLift = robot.getOutakeLiftsession();
            boolean forcedReset = false;
            outakeLift.closeGrabber();
            outake.idleOuttake();
            Timing.Timer timerInceput = new Timing.Timer(400,TimeUnit.MILLISECONDS);
            timerInceput.start();
            while (!timerInceput.done());
            outakeLift.idleArmGrabber();
            while(opModeIsActive()){
                gamepad2.setLedColor(217,65,148, 1000);
                gamepad1.setLedColor(179,250,60, 1000);
                double dist = (distanceSensor1.getDistance(DistanceUnit.CM) + distanceSensor2.getDistance(DistanceUnit.CM)) * 0.5;
                if(gamepad2.right_bumper){
                    drive.driveRobotCentric(
                            0,
                            -0.17 * Math.signum((int)((14-dist))),
                            0
                    );
                }
                else {
                    drive.driveRobotCentric(
                            -driverOp.getLeftX() * (outake.getMotorPosition() > 900 ? 0.75 : 1),
                            -driverOp.getLeftY() * (outake.getMotorPosition() > 900 ? 0.75 : 1),
                            -driverOp.getRightX() * ((outake.getMotorPosition() > 900 || intakeLift.getCurrentPosition() == IntakeLift.Position.EXTRACT) ? 0.75 : 1)
                    );
                }
                if(gamepad1.dpad_up){
                    outake.modifyPosition(true, false);
                }
                if(gamepad1.dpad_down){
                    outake.modifyPosition(false,gamepad1.share);
                    if(gamepad1.share)
                        forcedReset = true;
                }
                if(!gamepad1.share && gamepad1.dpad_down && forcedReset)
                {
                    outake.getMotorOuttake1().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    outake.getMotorOuttake2().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    forcedReset = false;
                }
                if(gamepad1.dpad_left) {
                    outake.extendOuttake();
                    outakeLift.idleArmGrabber();
                }
                if(gamepad1.dpad_right){
                    outake.idleOuttake();
                }
                if(gamepad1.ps)
                    outakeLift.up2ArmGrabber();
                if(gamepad1.options) {
                    if(opt == null || !opt.isAlive()) {
                        opt = new Thread(() -> {
                            outake.idleOuttake();
                            while (Math.abs(Outake.Position.IDLE.val-outake.getMotorPosition()) > 30);
                            outakeLift.specimenArmGrabber();
                            Timing.Timer timer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
                            timer.start();
                            while (timer.done()) ;
                            outake.retractOuttake();


                        });
                        opt.start();
                    }
                }
                /*if(gamepad1.left_bumper && debRB1.done()){
                    if(Math.abs(outake.getMotorPosition() - Outake.Position.SPECIMEN.val) < 35 ){
                        specimen = false;
                    } else{
                        specimen = true;
                    }
                    if
                    if(!specimen){

                    }
                    debRB1.start();
                }*/
                if(gamepad1.a && debA1.done()){

                    if(!intakeExtended){
                        intake.extendIntake();
                    }
                    else {
                        intake.retractIntake();
                    }

                    intakeExtended = !intakeExtended;
                    debA1.start();
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
                    if(y == null || !y.isAlive()){
                        y = new Thread(() -> {
                            outakeLift.openGrabber();
                            Timing.Timer timer = new Timing.Timer(150,TimeUnit.MILLISECONDS);
                            timer.start();
                            while (!timer.done() || gamepad1.y);
                            outakeLift.closeGrabber();
                        });
                        y.start();
                        debY.start();
                    }
                }
                if(gamepad1.right_stick_button && debStickR1.done()){
                    debStickR1.start();
                    if(shortcutRST == null || !shortcutRST.isAlive()) {

                        shortcutRST = new Thread(() -> {
                            outakeLift.downArmGrabber();
                            Timing.Timer timer = new Timing.Timer(450, TimeUnit.MILLISECONDS);
                            timer.start();
                            while (!timer.done()) ;
                            outake.retractOuttake();
                            while(outake.getMotorPosition() > 50);
                            outakeLift.openGrabber();

                            timer.start();
                            while (!timer.done()) ;

                            outakeLift.closeGrabber();

                            timer.start();
                            while (!timer.done()) ;
                            outake.idleOuttake();

                        });
                        shortcutRST.start();

                    }
                }
                if(gamepad1.left_bumper)
                    outakeLift.downArmGrabber();

                if(gamepad1.right_bumper)
                    outakeLift.upArmGrabber();
                if(gamepad1.left_stick_button)
                    outakeLift.idleArmGrabber();
                if(Math.abs(gamepad1.right_trigger - gamepad1.left_trigger) > 0.01){
                    double time = getTime();
                    double deltaTimp = time - lastTime;
                    intake.modifyPosition((gamepad1.right_trigger - gamepad1.left_trigger) * Config.kAIntake * deltaTimp);
                    lastTime = time;
                }
                telemetry.addData("Extended",intakeExtended);
                telemetry.update();
            }

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }
    }
}
