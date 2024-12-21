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
public class AutoFastRight extends LinearOpMode {
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
            sleep(2000);

            double dist = distanceSensor2.getDistance(DistanceUnit.CM);

            while (opModeIsActive() && Math.abs(17-dist) > 2){
                dist = distanceSensor2.getDistance(DistanceUnit.CM);

                drive.driveRobotCentric(
                        0,
                        0.17 * Math.atan(dist-17),
                        0
                );

                telemetry.addData("dist",dist);
                telemetry.update();
            }
            drive.driveRobotCentric(0,0,0);
            outtakeLift.downArmGrabber();
            sleep(800);
            outtakeLift.openGrabber();

            drive.driveRobotCentric(1,-0.6,0);
            sleep(1300);
            drive.stop();

            outtakeLift.idleArmGrabber();
            outtakeLift.closeGrabber();
            outake.retractOuttake();

            sleep(3000);

            outake.stopMotorOuttake();
            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }



    }

}
