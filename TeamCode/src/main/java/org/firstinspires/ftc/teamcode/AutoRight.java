package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Utils.KodiOdometry.ewp;
import static org.firstinspires.ftc.teamcode.Utils.KodiOdometry.wp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.types.PathType;
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
public class AutoRight extends LinearOpMode {

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



        IMU imu = hardwareMap.get(IMU.class, "imu");

        KodiOdometry kodiOdometry = new KodiOdometry(imu, backRightMotor, frontRightMotor);
        HolonomicOdometry odometry = kodiOdometry.getHolonomicOdometry();
        Path m_path = new Path(
                new StartWaypoint(odometry.getPose()),
                ewp(55,120,0)

        );

        m_path.setPathType(PathType.HEADING_CONTROLLED);
        waitForStart();
        try {
            while (opModeIsActive()) {
                //if (isStopRequested()) throw new InterruptedException()
                m_path.followPath(drive, odometry);
                telemetry.addData("PosX",odometry.getPose().getX());
                telemetry.addData("PosY",odometry.getPose().getY());
                telemetry.addData("Ang",odometry.getPose().getHeading());
                odometry.updatePose();

                telemetry.update();
                m_path.init();
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