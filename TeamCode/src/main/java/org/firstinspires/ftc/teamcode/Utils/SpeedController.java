package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.util.Range;

public class SpeedController {

    double speed = 0;

    double lastTime;

    double kA, kTan, kV;

    public double getTime(){
        return (double)System.currentTimeMillis() * 0.001;
    }

    public SpeedController(double accel, double maxV, double brakeDist){
        kA = accel;
        kTan = Math.PI * 0.5 / brakeDist;
        kV = maxV;

        lastTime = getTime();
    }

    public void updateCoef(double accel, double maxV, double brakeDist){
        kA = accel;
        kTan = Math.PI * 0.5 / brakeDist;
        kV = maxV;
    }

    public double getSpeed(double position){
        double time = getTime();
        speed += kA * (time - lastTime);
        speed = Range.clip(speed,0,kV);
        lastTime = time;
        return Range.clip(Math.atan(position * kTan) * speed,0,kV);
    }

    public void resetSpeed(){
        speed = 0;
    }


}
