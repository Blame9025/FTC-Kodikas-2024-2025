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

            // 1 SPECIMEN START

            outakeLift.closeGrabber();
            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();

            drive.driveRobotCentric(0,0.7,0);
            sleep(1100);
            drive.stop();

            // 1 SPECIMEN END

            // PRELUARE SASMPLE DE PE JOS !!! START !!!

            outakeLift.openGrabber();
            sleep(200);

            drive.driveRobotCentric(0,-0.5,0);
            sleep(400);
            drive.stop();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,60)
                    .goTo(68,60,180)
                    .goTo(68,135)
                    .goTo(90,135)
                    .goTo(90,135,180)
                    .goTo(90,20)
                    .goTo(90,135,180)
                    .goTo(115,135)
                    .goTo(115,20,180)
//                    .goTo(118,50,180)
//                    .goTo(129,135) // pozitii pentru al 3 lea sample testate MERG!
//                    .goTo(129,20,180)
//                    .goTo(129,50,180)
                    .execute();

            outakeLift.specimenArmGrabber(); // SAU UP2ARMGRABBER
            outake.retractOuttake();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            // PRELUARE SAMPLE DE PE JOS !!! END !!!

            // 2 SPECIMEN START

            sleep(1000);

            drive.driveRobotCentric(0,0.3,0);
            sleep(1200);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);
            outake.grabbSpecimen();
            sleep(300);


            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(-10,30,0)
                    .execute();

            outakeLift.up2ArmGrabber();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.7,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);

            // 2 SPECIMEN END

            // 3 SPECIMEN START

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(68,30,180)
                    .execute();

            outake.retractOuttake();
            outakeLift.specimenArmGrabber();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1000);

            drive.driveRobotCentric(0,0.3,0);
            sleep(1200);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(-18,30,0)
                    .execute();

            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.7,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);

            // 3 SPECIMEN END

            // 4 SPECIMEN START

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(68,30,180)
                    .execute();

            outake.retractOuttake();
            outakeLift.specimenArmGrabber();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1000);

            drive.driveRobotCentric(0,0.3,0);
            sleep(1200);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(-18,30,0)
                    .execute();

            outake.grabbSpecimen();
            outakeLift.up2ArmGrabber();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.7,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            intake.extendForceIntake(1);
            intakeLift.extractIntakeLift();
            sleep(400);

            // 4 SPECIMEN END

            // PARCARE START

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(80,30,125)
                    .execute();


            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            // PARCARE END

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
