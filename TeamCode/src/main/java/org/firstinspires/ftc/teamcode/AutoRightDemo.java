package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoRightDemo extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;
    Timing.Timer timer;

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
        try{
            initHW();

            Intake intake = robot.getIntakeSession();
            IntakeLift intakeLift = robot.getIntakeLiftSession();
            Outake outake = robot.getOutakeSession();
            OuttakeLift outakeLift = robot.getOutakeLiftsession();

            waitForStart();

            timer = new Timing.Timer(29, TimeUnit.SECONDS);
            timer.start();

            outakeLift.closeGrabber();
            outake.specimenBar();
            outakeLift.idleArmGrabber();

            outakeLift.closeGrabber();
            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,30)
                    .goTo(0,30,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested() || timer.done()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(600);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,35)
                    .goTo(76,70)
                    .goTo(76,70,0)
                    .goTo(76,135)
                    .goTo(87,135)
                    .goTo(87,135,0)
                    .goTo(87,20)
                    .goTo(87,135)
                    .goTo(118,135)
                    .goTo(118,20)
                    .goTo(118,60)
                    .goTo(118,60,180)
//                    .goTo(120,125)
//                    .goTo(135,125) // pozitii pentru al 3 lea sample testate MERG!
//                    .goTo(135,20)
//                    .goTo(135,30,0)
                    //.goTo(135,15)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested() || timer.done()) throw new InterruptedException();
            }

            outake.retractOuttake(); // !!!! DE VERIFICAT POZITIA !!!!
            sleep(500);
            outakeLift.openGrabber();
            sleep(500);
            outakeLift.specimenArmGrabber(); // pozitia de luat de la human player !!!! DE TESTAT SI MODIFICAT POZTITIA !!!!

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(125,40,180)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested() || timer.done()) throw new InterruptedException();
            }
            sleep(1200);

            drive.driveRobotCentric(0,0.3,0);
            sleep(800);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(600);
            outakeLift.autoUp();
            sleep(600);
            outake.grabbSpecimen();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(130,40)
                    .goTo(130,40,0)
                    .goTo(-10,30)
                    .goTo(-10,30,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested() || timer.done()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

//            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(130,40)
//                    .goTo(130,40,0)
//                    .goTo(-10,30)
//                    .goTo(-10,30,0)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());

            outakeLift.downArmGrabber();
            sleep(800);
            outake.retractOuttake();
            sleep(1000);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(60,15)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested() || timer.done()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(600);
            drive.stop();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(150,5)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested() || timer.done()) throw new InterruptedException();
            }

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
