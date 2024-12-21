package org.firstinspires.ftc.teamcode.AutoTimer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoParkRight extends LinearOpMode {

    private static final double time = 200;


    @Override
    public void runOpMode() throws InterruptedException {

        AutoMethods methods = new AutoMethods();

        sleep(28000);

        methods.goRight(1300);

    }
}
