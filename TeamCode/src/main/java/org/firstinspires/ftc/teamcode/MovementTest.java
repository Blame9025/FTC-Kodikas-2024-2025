package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Disabled
public class MovementTest extends LinearOpMode {

    Motor fL, bL, fR, bR;
    MecanumDrive drive;
    GamepadEx driverOp;

    IMU imu;

    final double CHASSIS_SPEED = 0.7;

    @Override
    public void runOpMode() throws InterruptedException {

        fL = new Motor(hardwareMap, "leftFrontMotor");
        bL = new Motor(hardwareMap, "leftRearMotor");
        fR = new Motor(hardwareMap, "rightFrontMotor");
        bR = new Motor(hardwareMap, "rightRearMotor");

        drive = new MecanumDrive(fL, fR, bL, bR);
        driverOp = new GamepadEx(gamepad1);
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        waitForStart(); // Asigură-te că metoda așteaptă pornirea opmode-ului

        while (opModeIsActive()) {
            drive.driveRobotCentric(
                    driverOp.getLeftX() * CHASSIS_SPEED,
                    driverOp.getLeftY() * CHASSIS_SPEED,
                    driverOp.getRightX() * CHASSIS_SPEED,
                    true
            );

            // Opțional: Dacă ambele sunt necesare, lasă și driveFieldCentric
            //drive.driveFieldCentric(
            //        driverOp.getLeftX() * CHASSIS_SPEED,
            //        driverOp.getLeftY() * CHASSIS_SPEED,
            //        driverOp.getRightX() * CHASSIS_SPEED,
            //        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS),
            //        true
            //);
        }
    }
}
