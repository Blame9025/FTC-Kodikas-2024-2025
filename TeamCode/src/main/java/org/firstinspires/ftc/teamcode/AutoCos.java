package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Utils.Intake;
@Autonomous
public class AutoCos extends LinearOpMode {

    void initHw()
    {
        DcMotor intakeMotor;

        intakeMotor = hardwareMap.dcMotor.get("intakeMotor");

    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
