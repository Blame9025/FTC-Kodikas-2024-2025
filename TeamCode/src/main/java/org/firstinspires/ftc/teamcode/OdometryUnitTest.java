package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;

@TeleOp
public class OdometryUnitTest extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    //KodiPursuit pp;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(
                hardwareMap, telemetry
        );
        drive = robot.getDriveSession();

        //pp = new KodiPursuit(hardwareMap,drive,telemetry);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();

            waitForStart();

            /*while (opModeIsActive()){
                telemetry.addData("x",pp.loc.x);
                telemetry.addData("y",pp.loc.y);
                telemetry.addData("t",pp.loc.theta);
                telemetry.update();
            }*/

            throw new InterruptedException();
        } catch (InterruptedException e) {
            //pp.loc.stop();
        }
    }
}
