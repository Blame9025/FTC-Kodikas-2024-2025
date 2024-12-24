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
public class OdometryUnitTest extends LinearOpMode {

    KodiLocalization loc;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        loc = new KodiLocalization(hardwareMap);
        loc.start();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();

            waitForStart();

            while (opModeIsActive()){
                telemetry.addData("x",loc.x);
                telemetry.addData("y",loc.y);
                telemetry.addData("t",loc.theta);
                telemetry.update();
            }

            throw new InterruptedException();
        } catch (InterruptedException e) {
            loc.stop();
        }
    }
}
