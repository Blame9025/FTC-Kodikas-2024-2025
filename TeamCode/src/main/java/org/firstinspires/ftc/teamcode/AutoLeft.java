package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiOdometry;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;


import java.util.concurrent.TimeUnit;

@Autonomous
public class AutoLeft extends LinearOpMode {
    Timing.Timer stop,delay;
    private static final long DEBUG_TIMER = 2000;
    private static final long delayTimer = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        KodikasRobot robot = new KodikasRobot(
                hardwareMap, telemetry
        );
        MecanumDrive drive = robot.getDriveSession();
        Intake intake = robot.getIntakeSession();
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        Outake outake = robot.getOutakeSession();
        OuttakeLift outtakeLift = robot.getOutakeLiftsession();
        DcMotor coreHexIntake = robot.getCoreHexIntake();
        Motor leftEncoder = robot.getBRightMotor();
        Motor rightEncoder = robot.getFRightMotor();
        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        KodiOdometry kodiOdometry = new KodiOdometry(imu, leftEncoder, rightEncoder);
        OdometrySubsystem odometry = new OdometrySubsystem(kodiOdometry.getHolonomicOdometry());
        waitForStart();
        try {

            PurePursuitCommand PureppCmd2 = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            0, 15, Math.toRadians(0),
                            0.6, 0.5, 30),
                    new GeneralWaypoint(
                            37.5, 15, Math.toRadians(0),
                            0.6, 0.5 , 30
                    ),
                    new GeneralWaypoint(
                            37.5,  60 , Math.toRadians(0),
                            0.6, 0.5 , 30
                    ),
                    new GeneralWaypoint(
                            37.5,  60 , Math.toRadians(180),
                            0.6, 0.5 , 30
                    ),
                    new GeneralWaypoint(
                            37.5,  75 , Math.toRadians(180),
                            0.6, 0.5 , 30
                    ),
                    new EndWaypoint()
            );
            PureppCmd2.schedule();
            while (!PureppCmd2.isFinished() && opModeIsActive()) {
                if (isStopRequested()) throw new InterruptedException();
            }

            outake.extendOuttake();


            PurePursuitCommand PureppCmd = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            0, 40, Math.toRadians(0),
                            0.6, 0.5, 30),
                    new GeneralWaypoint(
                            -90, 40, Math.toRadians(0),
                            0.6, 0.5 , 30
                    ),
                    new GeneralWaypoint(
                            -90,   15, Math.toRadians(0),
                            0.6, 0.5 , 30
                    ),
                    new GeneralWaypoint(
                            -90,   15, Math.toRadians(45),
                            0.6, 0.5 , 30
                    ),
                    new GeneralWaypoint(
                            -90,   0, Math.toRadians(45),
                            0.6, 0.5 , 30
                    ),
                    new EndWaypoint()
            );
            PureppCmd.schedule();
            while (!PureppCmd.isFinished() && opModeIsActive()) {
                if (isStopRequested()) throw new InterruptedException();
            }


        }
        catch (InterruptedException e){
            drive.stop();
            outake.stopMotorOuttake();
            intake.stop();
            coreHexIntake.setPower(0);
        }
    }
}
