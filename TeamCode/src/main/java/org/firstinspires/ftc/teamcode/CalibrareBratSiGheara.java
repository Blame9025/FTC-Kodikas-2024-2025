package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class CalibrareBratSiGheara extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servoClaw;
        Servo servoArmGrabber;

        servoClaw = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara


        waitForStart();

        servoClaw.setPosition(0.4);

        while(opModeIsActive()) ;
    }
}
