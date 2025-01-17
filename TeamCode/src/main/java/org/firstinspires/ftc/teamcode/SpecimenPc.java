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
public class SpecimenPc extends LinearOpMode {

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
            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();

            drive.driveRobotCentric(0,0.5,0);
            sleep(1300);
            drive.stop();


            outakeLift.openGrabber();
            sleep(200);
            outake.retractOuttake();
//            sleep(700);

            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(0,35)  // asta merge
                    .goTo(0,60) // netestat
                    .goTo(74,60,180)
                    .goTo(74,135)
                    .goTo(87,135)
                    .goTo(87,135,180)
                    .goTo(87,20)
                    .goTo(87,135,180)
                    .goTo(118,135)
                    .goTo(118,20,180)
//                    .goTo(118,50,180)
//                    .goTo(129,135) // pozitii pentru al 3 lea sample testate MERG!
//                    .goTo(129,20,180)
//                    .goTo(129,50,180)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

//            outake.retractOuttake(); // !!!! DE VERIFICAT POZITIA !!!!
//            sleep(500); // 2 specimen start
            outakeLift.openGrabber(); // 2 SPECIMEN START
//            sleep(250);
            outakeLift.specimenArmGrabber(); // pozitia de luat de la human player !!!! DE TESTAT SI MODIFICAT POZTITIA !!!!
            sleep(1000);

//            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(118,20,180)
//                    .execute();
//            while (!pp.finished()){
//                if(isStopRequested()) throw new InterruptedException();
//            }
//            sleep(800);

            drive.driveRobotCentric(0,0.3,0);
            sleep(800);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);
            outake.grabbSpecimen();
            sleep(300);
            outakeLift.autoUp();


            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(-10,30,0)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);
            outake.retractOuttake();
            outakeLift.specimenArmGrabber();
//            sleep(800); // 2 specimen end

            pp = new KodiPursuit(drive,telemetry,loc) // 3 specimen start
                    .goTo(68,30,180)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1000);

            drive.driveRobotCentric(0,0.3,0);
            sleep(1000);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);
            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();
            sleep(800);   // DE SCHIMBAT MAI IN JOS MAI EFICIENT

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(-18,30,0)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);
            outake.retractOuttake();
            sleep(800); // 3 specimen end

            pp = new KodiPursuit(drive,telemetry,loc) // 4 specimen start
                    .goTo(68,30,180)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }
            outakeLift.specimenArmGrabber();
            sleep(1200);

            drive.driveRobotCentric(0,0.3,0);
            sleep(1000);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);
            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(-25,30,0)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);
            outake.retractOuttake();
            sleep(800); // 4 specimen end

            pp = new KodiPursuit(drive,telemetry,loc) // parcare start
                    .goTo(125,10,0)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            } // parcare end





            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
