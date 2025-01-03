package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

@Autonomous
public class AutoRightKristee extends LinearOpMode {

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
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(20);
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
                    .goTo(65,70)
                    .goTo(65,135)
                    .goTo(93,135,90)
                    .goTo(93,30)
                    .goTo(93,135)
                    .goTo(125,135)
                    .goTo(125,30)
                    .goTo(125,60,180)
                    .goTo(90,30)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(15);
            while (dist.running());

            outake.retractOuttake(); // !!!! DE VERIFICAT POZITIA !!!!
            sleep(500);
            outakeLift.openGrabber();
            outakeLift.specimenArmGrabber(); // pozitia de luat de la human player !!!! DE TESTAT SI MODIFICAT POZTITIA !!!!
            sleep(500);
            outakeLift.closeGrabber();
            sleep(300);
            outakeLift.idleArmGrabber();
            sleep(600);

            outakeLift.closeGrabber();
            sleep(100);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(130,40,0)
                    .goTo(-10,30,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(20);
            while (dist.running());

            outake.specimenBar();
            sleep(1300);
            outakeLift.specimenArmGrabber();
            sleep(400);
            outakeLift.openGrabber();
            sleep(200);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(120,20)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outakeLift.idleArmGrabber();
            sleep(500);
            outakeLift.closeGrabber();
            sleep(5000);



            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
