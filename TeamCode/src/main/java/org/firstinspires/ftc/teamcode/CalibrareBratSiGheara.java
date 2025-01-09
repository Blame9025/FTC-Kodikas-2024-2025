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

        Servo servoArm;
        Servo servoArmGrabber;

        servoArm = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara


        waitForStart();
        double pos = 0;

        while(opModeIsActive()) {

            servoArmGrabber.setPosition(0);
            servoArm.setPosition(0);

        }
    }
}
