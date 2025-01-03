package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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

@TeleOp
public class AutoLeftDemo extends LinearOpMode {

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
        coreHexIntake = robot.getCoreHexIntake();

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

//            dist.run(17);
//            while (dist.running());

            outakeLift.closeGrabber();
            outake.specimenBar();
            outakeLift.idleArmGrabber();

            //sleep(3000);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(40,40,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(19);
            while (dist.running());

            outakeLift.specimenArmGrabber();
            sleep(800);
            outakeLift.openGrabber();
            sleep(600);
            outakeLift.closeGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,25)
                    .goTo(20,25,180)
                    .goTo(-56,25,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            intakeLift.extractIntakeLift();
            sleep(200);
            intake.extendIntake();

            sleep(1000);

            coreHexIntake.setPower(0.6);

            drive.driveRobotCentric(0,-0.3,0);
            sleep(1000);
            drive.stop();

            intake.retractIntake();
            coreHexIntake.setPower(-0.75);
            sleep(800);
            coreHexIntake.setPower(0);
//            outakeLift.openGrabber();
//            outake.retractOuttake();
//            sleep(1200);
//            outakeLift.closeGrabber();
//            sleep(200);
//            outake.extendOuttake();
//            outakeLift.idleArmGrabber();



            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
