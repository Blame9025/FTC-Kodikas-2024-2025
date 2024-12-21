package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.Range;

public class SpeedController {

    double speed = 0;

    double lastTime;

    double kA, kTan, kV;

    public double getTime(){
        return (double)System.currentTimeMillis() * 0.001;
    }

    public SpeedController(double accel, double brakeRate, double maxV){
        kA = accel;
        kTan = brakeRate;
        kV = maxV;

        lastTime = getTime();
    }

    public double getSpeed(double position){
        double time = getTime();
        speed += kA * (time - lastTime);
        speed = Range.clip(speed,0,kV);
        lastTime = time;
        return Math.atan(position * kTan) * speed;
    }


}
