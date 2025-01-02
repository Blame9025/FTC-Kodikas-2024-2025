package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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

@TeleOp
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
                    .goTo(70,70)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(19);
            while (dist.running());

            outakeLift.specimenArmGrabber();
            sleep(800);
            outakeLift.openGrabber();
            sleep(600);
            outakeLift.closeGrabber();
            outakeLift.downArmGrabber();
            sleep(600);
            outake.retractOuttake();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(70,125)
                    .goTo(97,125)
                    .goTo(97,125,90)
                    .goTo(97,20)
                    .goTo(97,125)
                    .goTo(120,125)
                    .goTo(120,20)
                    .goTo(120,125)
                    .goTo(135,125)
                    .goTo(135,20)
                    .goTo(135,30,0)
                    .goTo(135,15)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outake.grabbSpecimen(); // !!!! DE VERIFICAT POZITIA !!!!
            sleep(500);
            outakeLift.specimenArmGrabber(); // pozitia de luat de la human player !!!! DE TESTAT SI MODIFICAT POZTITIA !!!!
            outakeLift.openGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(135,4)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            sleep(600);

            outakeLift.closeGrabber();

            // Pana acum autonomia pune specimen ul din preload dupa impinge toate cele 3 sample la human player dupa se intoarce
            // si prinde un specimen pus de human player in colt ul din dreapta de tot aproape de perete !!!! DE TESTAT PE TEREN !!!!


            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
