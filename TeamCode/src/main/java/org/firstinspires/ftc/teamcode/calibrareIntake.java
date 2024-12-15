package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
@Disabled
public class calibrareIntake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servoIntake1,servoIntake2;
        DcMotor coreHexIntake;
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
        coreHexIntake = hardwareMap.dcMotor.get("leftRearMotor");

        servoIntake2.setDirection(Servo.Direction.REVERSE);
        servoIntake1.setDirection(Servo.Direction.FORWARD);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        coreHexIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        coreHexIntake.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        long startTime = System.currentTimeMillis();
        double pos = 1.0;
        double motorDirection = 1.0;
        while(opModeIsActive()) {
            servoIntake1.setPosition(pos);
            servoIntake2.setPosition(pos);
            coreHexIntake.setPower(motorDirection);
            if(System.currentTimeMillis() - startTime > 5000) {
                pos = 0.2;
                motorDirection = -1.0;
            }

        }
    }
}
