package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

@Autonomous
public class AutoRightAiCitinel extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;
    DcMotor coreHexIntake;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap, telemetry);
        drive = robot.getDriveSession();

        dist = new KodiDistance(hardwareMap,drive);

        loc = new KodiLocalization(hardwareMap);
        loc.start();

        coreHexIntake = robot.getCoreHexIntake();
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
            sleep(1500);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);
            outake.retractOuttake();
            sleep(600);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(45,45,45)
//                    .goTo(75,70)
//                    .goTo(75,70,0)
//                    .goTo(75,135)
//                    .goTo(93,135)
//                    .goTo(93,135,90)
//                    .goTo(93,20)
//                    .goTo(93,135)
//                    .goTo(125,135)
//                    .goTo(125,20)
//                    .goTo(125,60)
//                    .goTo(125,60,180)
//                    .goTo(125,40,180)
//                    .goTo(120,125)
//                    .goTo(135,125) // pozitii pentru al 3 lea sample testate MERG!
//                    .goTo(135,20)
//                    .goTo(135,30,0)
                    //.goTo(135,15)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            intakeLift.prepareIntakeLift();
            sleep(800);
            intake.extendIntake();
            sleep(800);
            intakeLift.extractIntakeLift();
            sleep(400);
            coreHexIntake.setPower(1);

            drive.driveRobotCentric(0.1,0.2,0);
            sleep(800);
            drive.stop();

            coreHexIntake.setPower(0);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(60,40,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            coreHexIntake.setPower(-0.75);
            sleep(750);
            coreHexIntake.setPower(1);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(60,40,45)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0.1,0.2,0);
            sleep(800);
            drive.stop();

            coreHexIntake.setPower(0);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(73,40,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            coreHexIntake.setPower(-0.75);
            sleep(750);
            coreHexIntake.setPower(1);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(73,40,45)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0.1,0.2,0);
            sleep(800);
            drive.stop();

            coreHexIntake.setPower(0);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(73,40,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            coreHexIntake.setPower(-0.75);
            sleep(800);
            coreHexIntake.setPower(0);
            intakeLift.prepareIntakeLift();
            sleep(600);
            intake.retractIntake();
            sleep(800);
            intakeLift.retractIntakeLift();
            sleep(450);
            outake.grabbSpecimen();
            sleep(500);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(68,40,180)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1200);

            drive.driveRobotCentric(0,0.3,0);
            sleep(800);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(400);
            outake.specimenBar();
            sleep(500);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(68,40,180)
                    .goTo(-20,40,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);
            outake.retractOuttake();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(68,40,180)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1200);

            drive.driveRobotCentric(0,0.3,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(68,40,180)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1200);

            drive.driveRobotCentric(0,0.3,0);
            sleep(800);
            drive.stop();

            outake.specimenBar();
            sleep(750);

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
