package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
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

    DcMotor leftEncoder, rightEncoder;
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

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        KodiOdometry kodiOdometry = new KodiOdometry(imu, leftEncoder, rightEncoder);
        OdometrySubsystem odometry = new OdometrySubsystem(kodiOdometry.getHolonomicOdometry());
        waitForStart();
        try {


            PurePursuitCommand PureppCmd = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            0, -90, Math.toRadians(0),
                            0.6, 0.5, 30)

            );
            PureppCmd.schedule();
            while (!PureppCmd.isFinished() && opModeIsActive()) {
                if (isStopRequested()) throw new InterruptedException();
            }

            intake.extendIntake();

            stop = new Timing.Timer(DEBUG_TIMER, TimeUnit.MILLISECONDS);
            stop.start();
            while (!stop.done()) {
                coreHexIntake.setPower(1);
            }

            intake.retractIntake();

            delay.start();
            while (!delay.done()) {
                coreHexIntake.setPower(-0.5);
            }


            outtakeLift.closeGrabber();

            PurePursuitCommand PureppCmd2 = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            -30, -90, Math.toRadians(45),
                            0.6, 0.5, 30)

            );
            PureppCmd2.schedule();
            while (!PureppCmd2.isFinished() && opModeIsActive()) {
                if (isStopRequested()) throw new InterruptedException();
            }

            outake.extendOuttake();

            if (outake.getCurrentPositionOuttake() == Outake.Position.EXTENDED) {
                outtakeLift.upArmGrabber();
            }

            PurePursuitCommand PureppCmd3 = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            -35, -90, Math.toRadians(45),
                            0.6, 0.5, 30)

            );
            PureppCmd3.schedule();
            while (!PureppCmd3.isFinished() && opModeIsActive()) {
                if (isStopRequested()) throw new InterruptedException();
            }

            outtakeLift.openGrabber();
        }
        catch (InterruptedException e){
            drive.stop();
            outake.stopMotorOuttake();
            intake.stop();
            coreHexIntake.setPower(0);
        }







    }
}
