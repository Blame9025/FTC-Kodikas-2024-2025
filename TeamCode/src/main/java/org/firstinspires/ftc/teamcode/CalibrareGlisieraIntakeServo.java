package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.KodiDistance;
import org.firstinspires.ftc.teamcode.Utils.KodiLocalization;
import org.firstinspires.ftc.teamcode.Utils.KodiPursuit;
import org.firstinspires.ftc.teamcode.Utils.KodikasRobot;
import org.firstinspires.ftc.teamcode.Utils.Intake;
import org.firstinspires.ftc.teamcode.Utils.Outake;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utils.IntakeLift;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

import java.util.concurrent.TimeUnit;

@TeleOp
public class CalibrareGlisieraIntakeServo extends LinearOpMode {
    private Servo servoIntake1,servoIntake2;
    private DcMotor coreHex;
    private Outake outtake;
    private Timing.Timer cooldown = new Timing.Timer(800, TimeUnit.MILLISECONDS);


    public void initHW(){
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
    }


    @Override
    public void runOpMode() throws InterruptedException {

            initHW();

            waitForStart();
            while(opModeIsActive()){



            }

            servoIntake1.setPosition(0.4);
            servoIntake2.setPosition(0.4);

    }
}
