package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class calibrareIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoIntake1,servoIntake2;
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");

        servoIntake2.setDirection(Servo.Direction.REVERSE);
        servoIntake1.setDirection(Servo.Direction.FORWARD);
        waitForStart();
        while(opModeIsActive()) {
            servoIntake1.setPosition(0.9);
            servoIntake2.setPosition(0.9);
        }
    }
}
