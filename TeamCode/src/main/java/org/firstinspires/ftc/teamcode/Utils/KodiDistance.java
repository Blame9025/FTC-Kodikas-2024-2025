package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class KodiDistance {

    DistanceSensor distanceSensor;
    MecanumDrive drive;
    SpeedController scSpecimen = new SpeedController(1,0.6,20);
    Thread positioningThread;

    public KodiDistance(HardwareMap hardwareMap, MecanumDrive drive){
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance2");
        this.drive = drive;
    }

    public KodiDistance run(double setDistanceCM){
        if(positioningThread == null || !positioningThread.isAlive()) {
            positioningThread = new Thread(() -> {
                double lastError = 2e9;
                while (!positioningThread.isInterrupted()) {
                    double error = distanceSensor.getDistance(DistanceUnit.CM) - setDistanceCM;

                    double speed = scSpecimen.getSpeed(error);

                    drive.driveRobotCentric(0, speed, 0);

                    if (Math.abs(error) < 1 && Math.abs(lastError - error) < Config.minimumRate)
                        positioningThread.interrupt();

                    lastError = error;
                }
            });
            positioningThread.start();
        }
        return this;
    }

    public void stop(){
        if(positioningThread != null){
            positioningThread.interrupt();
            drive.driveRobotCentric(0,0,0);
        }
    }

    public boolean running(){
        return positioningThread.isAlive();
    }

}
