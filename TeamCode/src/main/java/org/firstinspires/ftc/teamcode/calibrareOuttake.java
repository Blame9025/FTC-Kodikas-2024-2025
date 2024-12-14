package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utils.Config;

import java.util.concurrent.TimeUnit;

@TeleOp
public class calibrareOuttake extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoGrabber,servoArmGrabber;
        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        waitForStart();
        Timing.Timer delay = new Timing.Timer(5000, TimeUnit.MILLISECONDS);
        delay.start();
        boolean opened =false;
        while(opModeIsActive()) {
            servoGrabber.setPosition(opened ? Config.OUTTAKE_OPENPOS : Config.OUTTAKE_CLOSEDPOS);
            while (!delay.done() && opModeIsActive() && !opened);
            opened = true;
        }
    }
}
