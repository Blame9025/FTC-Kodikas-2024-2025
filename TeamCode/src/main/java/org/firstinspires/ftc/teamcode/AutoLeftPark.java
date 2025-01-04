package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

@TeleOp
public class AutoLeftPark extends LinearOpMode {

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

            waitForStart();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,100)
                    .goTo(0,100,90)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            OuttakeLift outakeLift = robot.getOutakeLiftsession();
            outakeLift.idleArmGrabber();
            sleep(500);

            drive.driveRobotCentric(0,0.5,0);
            sleep(1200);
            drive.stop();

            outakeLift.autoUp();
            sleep(200);

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
        }
    }
}
