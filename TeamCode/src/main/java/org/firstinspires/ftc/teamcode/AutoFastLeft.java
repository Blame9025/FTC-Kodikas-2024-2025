package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

@Autonomous
public class AutoFastLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        KodikasRobot robot = new KodikasRobot(hardwareMap, telemetry);
        DistanceSensor distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distance2");
        DistanceSensor distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distance2");
        OuttakeLift outtakeLift = robot.getOutakeLiftsession();
        Outake outake = robot.getOutakeSession();
        waitForStart();
        MecanumDrive drive = robot.getDriveSession();
        try{
            outtakeLift.closeGrabber();
            outake.specimenBar();
            outtakeLift.idleArmGrabber();
            sleep(1000);

            drive.driveRobotCentric(0.2,0.4,0);
            sleep(800);
            drive.stop();

            drive.driveRobotCentric(0,0,0.4);
            sleep(400);
            drive.stop();

            outake.extendOuttake();
            sleep(2500);

            outtakeLift.upArmGrabber();
            sleep(400);

            outtakeLift.openGrabber();
            sleep(200);

            outtakeLift.idleArmGrabber();
            outtakeLift.closeGrabber();
            outake.retractOuttake();

            /*drive.driveRobotCentric(0,0,0.4);
            sleep(2400);
            drive.stop();

            drive.driveRobotCentric(0,0.7,0);
            sleep(1600);
            drive.stop();

            drive.driveRobotCentric(0,0,-0.4);
            sleep(1200);
            drive.stop();

            double dist = distanceSensor2.getDistance(DistanceUnit.CM);

            while (opModeIsActive() && Math.abs(14-dist) > 2){
                dist = distanceSensor2.getDistance(DistanceUnit.CM);

                drive.driveRobotCentric(
                        0,
                        0.17 * Math.atan(dist-14),
                        0
                );

                telemetry.addData("dist",dist);
                telemetry.update();
            }

            outtakeLift.upArmGrabber();
            sleep(100);*/

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }



    }

}
