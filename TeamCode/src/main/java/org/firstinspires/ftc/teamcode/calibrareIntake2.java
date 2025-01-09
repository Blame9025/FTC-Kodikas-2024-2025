package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class calibrareIntake2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoArm;

        servoArm = hardwareMap.servo.get("servoGrabber");
        waitForStart();
        double pos = 0.0    ;
        servoArm.setPosition(pos);
        while (opModeIsActive());

    }
}
