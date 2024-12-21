package org.firstinspires.ftc.teamcode.AutoTimer;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoLeft extends LinearOpMode {

    private static final double time = 200;
    //private static final long nineteenDegree = 600;
    KodikasRobot robot;

    Timing.Timer delay = new Timing.Timer(3000, TimeUnit.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {


        Intake intake = robot.getIntakeSession();
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        Outake outake = robot.getOutakeSession();
        OuttakeLift outakeLift = robot.getOutakeLiftsession();
        AutoMethods methods = new AutoMethods();

        outakeLift.closeGrabber();
        methods.diagonalForwardRight(1000);
        methods.clockwiseRight(600);

        outake.specimenBar();
        outakeLift.idleArmGrabber();

        delay.start();
        while (!delay.done()){
            telemetry.addData("delay: ", "TRUE");
            telemetry.update();
        }

        outakeLift.specimenArmGrabber();
        sleep(700);

        outakeLift.openGrabber();
        outakeLift.downArmGrabber();
        sleep(1000);

        outake.retractOuttake();
        sleep(2000);

        methods.clockwiseRight(600);
        methods.goRight(1000);

    }
}
