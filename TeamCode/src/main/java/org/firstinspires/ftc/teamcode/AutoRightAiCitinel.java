package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;
import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Outake;
import org.firstinspires.ftc.teamcode.Utils.OuttakeLift;

@Autonomous
public class AutoRightAiCitinel extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;
    DcMotor coreHexIntake;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap, telemetry);
        drive = robot.getDriveSession();

        dist = new KodiDistance(hardwareMap,drive);

        loc = new KodiLocalization(hardwareMap);
        loc.start();

        coreHexIntake = robot.getCoreHexIntake();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();

            Intake intake = robot.getIntakeSession();
            IntakeLift intakeLift = robot.getIntakeLiftSession();
            Outake outake = robot.getOutakeSession();
            OuttakeLift outakeLift = robot.getOutakeLiftsession();

            waitForStart();

            outakeLift.closeGrabber();
            sleep(200);
            outake.specimenBar();
            outakeLift.up2ArmGrabber();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc) // 1 START
                    .goTo(-14,40,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop(); // 1 END

            outakeLift.openGrabber();
            sleep(300);

            pp = new KodiPursuit(drive,telemetry,loc) // 2 START DE PE JOS
                    .goTo(70,59,45) // verificat cu ruleta, neverificat unghiul
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            intakeLift.prepareIntakeLift();
            sleep(600);
            intake.extendIntake();
            sleep(800);
            intakeLift.extractIntakeLift();
            sleep(400);
            coreHexIntake.setPower(1);

            drive.driveRobotCentric(0,0.2,0);
            sleep(800);
            drive.stop();

            coreHexIntake.setPower(0);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(70,59,-45) // verificat cu ruleta, neverificat unghiul
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            coreHexIntake.setPower(-1);
            sleep(750);
            coreHexIntake.setPower(1); // 2 END DE PE JOS

            pp = new KodiPursuit(drive,telemetry,loc)  // 3 START DE PE JOS
                    .goTo(95,59,45)// verificat prin poze, neverificat unghiul
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.2,0);
            sleep(800);
            drive.stop();

            coreHexIntake.setPower(0);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(95,59,-45)// verificat prin poze, neverificat unghiul
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            coreHexIntake.setPower(-1);
            sleep(750);
            coreHexIntake.setPower(1); // 3 END DE PE JOS

            pp = new KodiPursuit(drive,telemetry,loc) // 4 START DE PE JOS
                    .goTo(120,59,45)// verificat prin poze, neverificat unghiul
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.2,0);
            sleep(800);
            drive.stop();

            coreHexIntake.setPower(0);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(120,59,-45)// verificat prin poze, neverificat unghiul
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            coreHexIntake.setPower(-1);
            sleep(800); // 4 END DE PE JOS
            coreHexIntake.setPower(0);
            intakeLift.prepareIntakeLift(); // 2 START
            sleep(600);
            intake.retractIntake();
            sleep(800);
            intakeLift.retractIntakeLift();
            sleep(450);
            outake.grabbSpecimen();
            sleep(500);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(72,40,180) // verificat cu ruleta
                    .goTo(72,40,180)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            sleep(1200);

            drive.driveRobotCentric(0,0.2,0);
            sleep(800);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(400);
            outake.specimenBar();
            sleep(500);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(72,40,180)
                    .goTo(-26,40,0)
                    .goTo(-26,40,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300); // 2 END

            pp = new KodiPursuit(drive,telemetry,loc) // 3 START
                    .goTo(72,40,180)
                    .goTo(72,40,180)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            outake.grabbSpecimen();
            sleep(2000);


            drive.driveRobotCentric(0,0.3,0);
            sleep(800);
            drive.stop();

            outakeLift.closeGrabber();
            sleep(300);
            outake.specimenBar();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(72,40,180)
                    .goTo(-33,40,0)
                    .goTo(-33,40,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0,0.5,0);
            sleep(800);
            drive.stop();

            outakeLift.openGrabber();
            sleep(300);

            //PARCARE START

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(0,40,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }
            outake.grabbSpecimen();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(100,7,0)
                    .execute();
            while (!pp.finished() && opModeIsActive()){
                if(isStopRequested()) throw new InterruptedException();
            }

            drive.driveRobotCentric(0.3,0,0); // se duce in dreapta ca sa se parcheze in colt
            sleep(800);
            drive.stop();

            //PARCARE END

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
