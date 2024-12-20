package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp
public class TestDistance extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor distanceSensor1 = hardwareMap.get(DistanceSensor.class, "distance1");
        DistanceSensor distanceSensor2 = hardwareMap.get(DistanceSensor.class, "distance2");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("Range 1", String.format("%.01f cm", distanceSensor1.getDistance(DistanceUnit.CM)));
            telemetry.addData("Range 2", String.format("%.01f cm", distanceSensor2.getDistance(DistanceUnit.CM)));
            telemetry.update();
        }
    }
}
