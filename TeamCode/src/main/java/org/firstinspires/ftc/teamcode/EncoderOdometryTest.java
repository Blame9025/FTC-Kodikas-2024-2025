package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Utils.Config;
import org.firstinspires.ftc.teamcode.Utils.KodiOdometry;

@Autonomous(name = "Test Encoder Position with rightRearMotor")
public class EncoderOdometryTest extends LinearOpMode { // 

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightEncoder = hardwareMap.dcMotor.get("rightFrontMotor"); // Using rightRearMotor as vertical encoder
        DcMotor backEncoder = hardwareMap.dcMotor.get("rightRearMotor"); // Horizontal encoder
        IMU imu = hardwareMap.get(IMU.class, "imu");

        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize odometry system
        KodiOdometry kodiOdometry = new KodiOdometry(imu, rightEncoder, backEncoder);

        waitForStart();

        while (opModeIsActive()) {
            // Update odometry position
            kodiOdometry.getHolonomicOdometry().updatePose();

            // Get the current encoder positions
            int rightVerticalPosition = rightEncoder.getCurrentPosition();
            int horizontalPosition = backEncoder.getCurrentPosition();

            // Convert encoder ticks to centimeters (example conversion factor)
            double rightVerticalPositionCm = rightVerticalPosition * Config.TICKS_TO_CM;
            double horizontalPositionCm = horizontalPosition * Config.TICKS_TO_CM;

            // Display the encoder positions on telemetry
            telemetry.addData("Right Vertical Encoder Position (ticks)", rightVerticalPosition);
            telemetry.addData("Right Vertical Encoder Position (cm)", rightVerticalPositionCm);
            telemetry.addData("Horizontal Encoder Position (ticks)", horizontalPosition);
            telemetry.addData("Horizontal Encoder Position (cm)", horizontalPositionCm);
            telemetry.update();
        }
    }
}
