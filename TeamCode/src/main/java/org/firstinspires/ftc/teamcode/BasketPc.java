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
public class BasketPc extends LinearOpMode {

    KodikasRobot robot;
    MecanumDrive drive;
    KodiPursuit pp;
    KodiLocalization loc;
    KodiDistance dist;
    DcMotor coreHex;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new KodikasRobot(hardwareMap, telemetry);
        drive = robot.getDriveSession();
        coreHex = robot.getCoreHexIntake();

        dist = new KodiDistance(hardwareMap,drive);

        loc = new KodiLocalization(hardwareMap);
        loc.start();
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

            outakeLift.closeGrabber(); // sample 1 start
//            outake.specimenBar();
//            outakeLift.idleArmGrabber();

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,31,-45)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outake.extendOuttake();
            outakeLift.up2ArmGrabber();
            sleep(800);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,45,-45) // x = 18
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.openGrabber();
            sleep(350);



            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,29,-85)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.closeGrabber();
//            outakeLift.downArmGrabber();
            outake.grabbSpecimen(); // sample 1 end
            sleep(1500);


            coreHex.setPower(1);  // sample 2 start
            intakeLift.extractIntakeLiftAuto();
            intake.extendIntake(0.7);
            sleep(1000);

            drive.driveRobotCentric(0,-0.2,0);
            sleep(900);
            drive.stop();
            sleep(200);

            coreHex.setPower(0);
            intakeLift.prepareIntakeLift();
            sleep(1000);
            intake.retractIntake(0.2);
            sleep(1000);
            intakeLift.retractIntakeLift();
//            outakeLift.closeGrabber();
            sleep(600);


            coreHex.setPower(-0.75);
//            outakeLift.downArmGrabber();
//            sleep(500);
            outakeLift.openGrabber();
            sleep(400);
            coreHex.setPower(1);
            sleep(400);
            coreHex.setPower(-0.75);
            sleep(400);
            coreHex.setPower(0);
            sleep(300);
            outake.retractOuttake();
            sleep(550);
            outakeLift.closeGrabber();
            sleep(300);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,31,-45)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outake.extendOuttake();
            outakeLift.up2ArmGrabber();
            sleep(1000);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,45,-45) // x = 18
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.openGrabber();
            sleep(350);

            pp = new KodiPursuit(drive,telemetry,loc)
                    .goTo(20,31,-45)
                    .execute();
            while (!pp.finished()){
                if(isStopRequested()) throw new InterruptedException();
            }

            outakeLift.closeGrabber();
            outakeLift.downArmGrabber();
            outake.grabbSpecimen();
            sleep(1000); // sample 2 end

//            pp = new KodiPursuit(drive,telemetry,loc) // sample 3 start
////                    .goTo(15,12)
////                    .goTo(15,12,0)
////                    .goTo(130,-5)
////                    .goTo(130,-5,180)
//                    .goTo(30,47.5,90)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            intake.extendIntake();
//            intakeLift.extractIntakeLiftAuto();
//            sleep(1000);
//            coreHex.setPower(1);
//
//            drive.driveRobotCentric(0,0.2,0);
//            sleep(600);
//            drive.stop();
//
//            coreHex.setPower(0);
//            intake.retractIntake();
//            intakeLift.prepareIntakeLift();
//            sleep(1000);
//            intakeLift.retractIntakeLift();
////            outakeLift.closeGrabber();
//            sleep(600);
//            coreHex.setPower(-0.75);
////            outakeLift.downArmGrabber();
////            sleep(500);
//            outakeLift.openGrabber();
//            coreHex.setPower(0);
//            sleep(300);
//            outake.retractOuttake();
//            sleep(550);
//            outakeLift.closeGrabber();
//            sleep(300);
//
//            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(35,50,-45)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            outake.extendOuttake();
//            sleep(1200);
//            outakeLift.up2ArmGrabber();
//            sleep(400);
//
//            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(35,35,-45)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            outakeLift.openGrabber();
//            sleep(300);
//
//            pp = new KodiPursuit(drive,telemetry,loc)
//                    .goTo(35,50,-45)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            outake.grabbSpecimen();
//            sleep(1000); // sample 3 end
//
//            pp = new KodiPursuit(drive,telemetry,loc) // parcare start
//                    .goTo(100,0,180)
//                    .goTo(100,-25)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            outake.retractOuttake();
//            sleep(800);
//            outakeLift.upArmGrabber(); // parcare end
//

//            pp = new KodiPursuit(drive,telemetry,loc) //START SECTION SAMPLE 1 DE PE JOS
//                    .goTo(30,30)
//                    .goTo(30,30,180)
//                    //.goTo(30,60)
//                    .execute();
//            while (!pp.finished() && opModeIsActive());
//
//            outakeLift.downArmGrabber();
//            sleep(700);
//            outakeLift.openGrabber();
//            sleep(400);
//            outake.retractOuttake();
//            sleep(1500);
//            intakeLift.prepareIntakeLift();
//            sleep(500);
//            intake.autoPos();
//            sleep(1000);
//            coreHex.setPower(1);
//
//            drive.driveRobotCentric(0,0.2,0);
//            sleep(800);
//            drive.stop();
//
//            coreHex.setPower(0);
//            intakeLift.prepareIntakeLift();
//            intake.retractIntake();
//            sleep(1000);
//
//            coreHex.setPower(-0.75);
//            sleep(800);
//
//            outakeLift.closeGrabber();
//            sleep(300); //END SECTION SAMPLE 1 DE PE JOS*/

            throw new InterruptedException();
        } catch (InterruptedException e) {
            pp.kill();
            dist.stop();
        }
    }
}
