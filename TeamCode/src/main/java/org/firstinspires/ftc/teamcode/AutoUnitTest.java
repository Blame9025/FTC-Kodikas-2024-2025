package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;

@TeleOp
public class AutoUnitTest extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(
                hardwareMap, telemetry
        );
        drive = robot.getDriveSession();

        loc = new KodiLocalization(hardwareMap);
        loc.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();

            waitForStart();

            pp = new KodiPursuit(hardwareMap,drive,telemetry,loc)
                    .goTo(0,0)
                    .goTo(70,70)
                    .goTo(70,130)
                    .execute();
            while (!pp.finished() && opModeIsActive());

            sleep(3000);

            pp = new KodiPursuit(hardwareMap,drive,telemetry,loc)
                    .goTo(70,130)
                    .goTo(100,130)
                    .goTo(100,30)
                    .goTo(0,0)
                    .execute();
            while (!pp.finished() && opModeIsActive());


            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
        }
    }
}
