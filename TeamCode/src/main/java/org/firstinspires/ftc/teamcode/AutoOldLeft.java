package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.arcrobotics.ftclib.util.Timing;
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

@Autonomous
public class AutoOldLeft extends LinearOpMode {

    DcMotor leftEncoder, rightEncoder;
    Timing.Timer stop,delay;
    private static final long DEBUG_TIMER = 2000;
    private static final long delayTimer = 500;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        KodikasRobot robot = new KodikasRobot(
                hardwareMap, telemetry
        );
        MecanumDrive drive = robot.getDriveSession();
        Intake intake = robot.getIntakeSession();
        IntakeLift intakeLift = robot.getIntakeLiftSession();
        Outake outake = robot.getOutakeSession();
        OuttakeLift outtakeLift = robot.getOutakeLiftsession();
        DcMotor coreHexIntake = robot.getCoreHexIntake();

        Motor backRightMotor, frontRightMotor;

        backRightMotor = robot.getBRightMotor();
        frontRightMotor = robot.getFRightMotor();

        backRightMotor.resetEncoder();
        frontRightMotor.resetEncoder();

        IMU imu = hardwareMap.get(IMU.class, "imu");

        KodiOdometry kodiOdometry = new KodiOdometry(imu, backRightMotor, frontRightMotor);
        OdometrySubsystem odometry = new OdometrySubsystem(kodiOdometry.getHolonomicOdometry());
        waitForStart();
        try {

            PurePursuitCommand PureppCmd = new PurePursuitCommand(
                    drive, odometry,
                    new StartWaypoint(odometry.getPose()),
                    new GeneralWaypoint(
                            0, 5, 0,
                            0.6, 0.5, 30
                    ),

                    new EndWaypoint(
                            0, 10, 0,
                            0.6, 0.5, 30, 0.8, 1)
            );
            PureppCmd.schedule();

            while (!PureppCmd.isFinished() && opModeIsActive()) {
                //if (isStopRequested()) throw new InterruptedException();
                PureppCmd.execute();
                telemetry.addData("PosX",odometry.getPose().getX());
                telemetry.addData("PosY",odometry.getPose().getY());
                telemetry.addData("Ang",odometry.getPose().getHeading());
                odometry.update();

                telemetry.update();
            }
            //telemetry.addData("S-A TERMINAT TASK",true);
            //telemetry.update();
            drive.stop();
            //PureppCmd.end(true);
            if (opModeIsActive()) sleep(1000);
            throw  new InterruptedException();
        }
        catch (InterruptedException e){
            drive.stop();
            outake.stopMotorOuttake();
            intake.stop();
            coreHexIntake.setPower(0);
        }
    }
}