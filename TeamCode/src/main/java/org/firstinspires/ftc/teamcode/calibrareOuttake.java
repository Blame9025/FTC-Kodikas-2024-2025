package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class calibrareOuttake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoGrabber,servoArmGrabber;
        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        waitForStart();
        while(opModeIsActive()) {
            servoGrabber.setPosition(0.5);
            servoArmGrabber.setPosition(0.5);
        }
    }
}
