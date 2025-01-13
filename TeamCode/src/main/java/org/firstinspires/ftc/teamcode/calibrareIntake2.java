package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Utils.Config;
@TeleOp

public class calibrareIntake2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoClaw;

        servoClaw = hardwareMap.servo.get("servoGrabber");
        waitForStart();
        servoClaw.setPosition(Config.DESCHIDERE_CLESTE);
        while (opModeIsActive());

    }
}
