package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestIntake extends LinearOpMode {

    DcMotor motorIntake;

    public void initHW(){

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motorIntake = hardwareMap.dcMotor.get("coreHexGlisiera");
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        try{
            initHW();

            waitForStart();

            while(opModeIsActive()){
                motorIntake.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            }

            throw new InterruptedException();
        } catch (InterruptedException e) {
            motorIntake.setPower(0);
        }
    }
}
