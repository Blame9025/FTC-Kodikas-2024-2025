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
public class AutoLeftAlbertDemo extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;
    DcMotor coreHex;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap, telemetry);
        drive = robot.getDriveSession();
        coreHex = robot.getCoreHexIntake();

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
//                    .goTo(60,0) // vechi care merge
//                    .goTo(60,0,-45)
//                    .goTo(35,35,-45)
                    .goTo(35,50,-45)
                    .goTo(35,35,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outake.extendOuttake();
            sleep(1500);
            outakeLift.up2ArmGrabber();
            sleep(300);

//            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(10,28)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());

            outakeLift.openGrabber();
            sleep(300);
            outakeLift.idleArmGrabber();
            sleep(500);
            outakeLift.closeGrabber();
            sleep(300);
            outake.grabbSpecimen();
            sleep(1000);

            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(15,12)
//                    .goTo(15,12,0)
//                    .goTo(130,-5)
//                    .goTo(130,-5,180)
                    .goTo(62,11,90)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            intakeLift.autoServoIntake();
            sleep(600);
            coreHex.setPower(1);


            drive.driveRobotCentric(0,0.2,0);
            sleep(600);
            drive.stop();

            coreHex.setPower(0);
            intakeLift.retractIntakeLift();
            sleep(400);
            coreHex.setPower(-0.75);
            sleep(700);
            coreHex.setPower(0);
            outakeLift.openGrabber();
            sleep(300);
            outakeLift.downArmGrabber();
            sleep(500);
            outake.retractOuttake();
            sleep(800);
            outakeLift.closeGrabber();
            sleep(400);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(35,50,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outake.extendOuttake();
            sleep(1200);
            outakeLift.idleArmGrabber();
            sleep(600);

            pp = new KodiPursuit(drive,telemetry,loc)

                    .goTo(35,35,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outakeLift.autoUp();
            sleep(300);
            outakeLift.openGrabber();
            sleep(400);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(35,50,-45)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outakeLift.idleArmGrabber();
            sleep(400);
            outake.retractOuttake();
            sleep(1000);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(100,0,180)
                    .goTo(100,7,180)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            outakeLift.up2ArmGrabber();
            sleep(300);

//            outakeLift.autoUp();
//            sleep(200);


//            pp = new KodiPursuit(drive,telemetry,loc) //START SECTION SAMPLE 1 DE PE JOS
//                    .goTo(30,30)
//                    .goTo(30,30,180)
//                    //.goTo(30,60)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            outakeLift.downArmGrabber();
//            sleep(700);
//            outakeLift.openGrabber();
//            sleep(400);
//            outake.retractOuttake();
//            sleep(1500);
//            intakeLift.prepareIntakeLift();
//            sleep(500);
//            intake.autoPos();
//            sleep(1000);
//            coreHex.setPower(1);
//
//            drive.driveRobotCentric(0,0.2,0);
//            sleep(800);
//            drive.stop();
//
//            coreHex.setPower(0);
//            intakeLift.prepareIntakeLift();
//            intake.retractIntake();
//            sleep(1000);
//
//            coreHex.setPower(-0.75);
//            sleep(800);
//
//            outakeLift.closeGrabber();
//            sleep(300); //END SECTION SAMPLE 1 DE PE JOS

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
