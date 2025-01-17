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
public class BasketPc extends LinearOpMode {

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

            outakeLift.closeGrabber(); // sample 1 start
            outakeLift.idleArmGrabber();
//            outake.specimenBar();
//            outakeLift.idleArmGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(22,45.5,-45) // x = 18
                    .execute();

            outake.extendOuttake();
            while (outake.isBusy()){
                if(isStopRequested()) throw new InterruptedException();
            }
            outakeLift.upArmGrabber();
            sleep(300);
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.openGrabber();
            sleep(600); // sample 1 end

            outakeLift.idleArmGrabber();
            sleep(600);


            // sample 2 start

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(23,26,-93)
                    .execute();

            outakeLift.closeGrabber();
            outake.grabbSpecimen();
//            outakeLift.upArmGrabber();

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }


            intakeLift.extractIntakeLift();
            intake.extendForceIntake(1);
            sleep(630);
            intakeLift.extractIntakeLift();
            sleep(300);
//            sleep(1000);
            coreHex.setPower(1);
            sleep(700);

            drive.driveRobotCentric(0,-0.25,0);
            sleep(1200);
            drive.stop();
            sleep(500);

            coreHex.setPower(0);
            intakeLift.prepareIntakeLift();
            sleep(400);
            intake.retractForceIntake(1);
            sleep(800); // sleep de 400 NU MERGE
            intakeLift.retractIntakeLift();
//            outakeLift.closeGrabber();
            sleep(400);


            coreHex.setPower(-1);
//            outakeLift.downArmGrabberAuto();
            sleep(500);
            outakeLift.openGrabber();
            coreHex.setPower(1);
            sleep(500);
            coreHex.setPower(-1);
            sleep(700);
            coreHex.setPower(0);

            outakeLift.downArmGrabber();
            sleep(1000);
            outake.retractOuttake();
            sleep(500);
            outakeLift.closeGrabber();
            sleep(600);
            outakeLift.idleArmGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(21,43.5,-47) // x = 18
                    .execute();

            outake.extendOuttake();
            while (outake.isBusy()){
                if(isStopRequested()) throw new InterruptedException();
            }
            outakeLift.upArmGrabber();
            sleep(400);

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.openGrabber();
            sleep(300);

            outakeLift.idleArmGrabber();
            sleep(300);
            outake.grabbSpecimen();// sample 2 end

            pp = new KodiPursuit(drive,telemetry,loc) // sample 3 start
                    .goTo(23,47,-83)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }


            intake.extendForceIntake(1);
            sleep(630);
            intakeLift.extractIntakeLift();
            sleep(400);
//            sleep(1000);
            coreHex.setPower(1);
            sleep(700);

            drive.driveRobotCentric(0,-0.25,0);
            sleep(1700);
            drive.stop();
            sleep(500);

            coreHex.setPower(0);
            intakeLift.prepareIntakeLift();
            sleep(400);
            intake.retractForceIntake(1);
            sleep(800);
            intakeLift.retractIntakeLift();
//            outakeLift.closeGrabber();
            sleep(400);


            coreHex.setPower(-1);
//            outakeLift.downArmGrabberAuto();
            sleep(500);
            outakeLift.openGrabber();
            coreHex.setPower(1);
            sleep(500);
            coreHex.setPower(-1);
            sleep(700);
            coreHex.setPower(0);

            outakeLift.downArmGrabber();
            sleep(1000);
            outake.retractOuttake();
            sleep(500);
            outakeLift.closeGrabber();
            sleep(600);
//            outakeLift.idleArmGrabber();
//            sleep(300);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(21,43.5,-47) // x = 18
                    .execute();

            outake.extendOuttake();
            sleep(300);
            outakeLift.idleArmGrabber();

            while (outake.isBusy()){
                if(isStopRequested()) throw new InterruptedException();
            }
            outakeLift.upArmGrabber();
            sleep(400);

            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.openGrabber();
            sleep(500); // sample 3 end

            outakeLift.idleArmGrabber();
            outake.retractOuttake();
            sleep(1500);

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
