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

@Autonomous(name = "AlbertSpecimenParkDemo")
public class AutoRightAlbertDemo extends LinearOpMode {

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

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,30)
                    .goTo(0,30,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            dist.run(5);
            while (dist.running());

            sleep(1000);
            outakeLift.openGrabber();
            sleep(600);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,30)
                    .goTo(125,10)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            sleep(200);

            outakeLift.idleArmGrabber();
            sleep(600);
            outake.retractOuttake();
            sleep(1000);

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
