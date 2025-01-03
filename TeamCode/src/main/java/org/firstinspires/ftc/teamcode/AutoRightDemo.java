package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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

@Autonomous
public class AutoRightDemo extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;

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

            outakeLift.closeGrabber();
            outake.specimenBar();
            outakeLift.idleArmGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,30)
                    .goTo(0,30,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(23);
            while (dist.running());

            outakeLift.specimenArmGrabber();
            sleep(800);
            outakeLift.openGrabber();
            sleep(600);
            outakeLift.idleArmGrabber();
            sleep(600);
            outake.retractOuttake();
            sleep(1000);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(60,70)
                    .goTo(60,135)
                    .goTo(93,135)
                    .goTo(93,135,90)
                    .goTo(93,30)
                    .goTo(93,135)
                    .goTo(125,135)
                    .goTo(125,50)
                    .goTo(125,30)
                    .goTo(125,30,180)
//                    .goTo(120,125)
//                    .goTo(135,125) // pozitii pentru al 3 lea sample testate MERG!
//                    .goTo(135,20)
//                    .goTo(135,30,0)
                    //.goTo(135,15)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            sleep(2000);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(125,25)
                    .goTo(125,25,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(15);
            while (dist.running());

            outake.retractOuttake(); // !!!! DE VERIFICAT POZITIA !!!!
            sleep(500);
            outakeLift.openGrabber();
            sleep(500);
            outakeLift.specimenArmGrabber(); // pozitia de luat de la human player !!!! DE TESTAT SI MODIFICAT POZTITIA !!!!
            sleep(500);
            outakeLift.closeGrabber();
            sleep(500);
            outakeLift.idleArmGrabber();

            sleep(600);

            outakeLift.closeGrabber();
            sleep(100);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(130,40)
                    .goTo(130,40,0)
                    .goTo(-10,30)
                    .goTo(-10,30,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(23);
            while (dist.running());

            outake.specimenBar();
            sleep(1300);
            outakeLift.idleArmGrabber();
            sleep(600);
            outakeLift.specimenArmGrabber();
            sleep(400);
            outakeLift.openGrabber();
            sleep(200);
            outakeLift.idleArmGrabber();
            sleep(500);
            outakeLift.closeGrabber();
            sleep(300);



            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
