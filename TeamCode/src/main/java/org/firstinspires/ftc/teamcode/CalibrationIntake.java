package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class CalibrationIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoIntake1,servoIntake2;
        DcMotor coreHexIntake;
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");


        waitForStart();
        double pos = 0;

        while(opModeIsActive()) {
            servoIntake1.setPosition(pos);
            servoIntake2.setPosition(pos);

        }
    }
}
