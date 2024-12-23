package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class KodiPursuit {

    HardwareMap hMap;
    public KodiLocalization loc;

    Thread pursuitThread;

    public ArrayList<Point> waypoints = new ArrayList<>();

    public Point lastPoint, targetPoint;

    public MecanumDrive drive;

    Telemetry telemetry;

    /**
     *
     * KodiPursuit pp = new KodiPursuit(hMap,drive)
     *                      .goTo(loc.x,loc.y)
     *                      .goto(x1,y1,theta1)
     *                      .goto(x2,y2,theta2)
     *                      ....................
     *                      .execute()
     *
     * while(!pp.finished() && opModeIsActive());
     * pp.kill();
     *
     */

    public KodiPursuit(HardwareMap hMap, MecanumDrive drive, Telemetry telemetry, KodiLocalization loc){
        this.hMap = hMap;
        this.drive = drive;
        this.telemetry = telemetry;
        this.loc = loc;
    }

    public KodiPursuit goTo(double x, double y){
        waypoints.add(new Point(x,y,Double.NaN));
        return this;
    }

    public KodiPursuit goTo(double x, double y, double theta){
        double angle = (int)(theta + 3600) % 360;
        waypoints.add(new Point(x,y,angle));
        return this;
    }

    public static double minAbs(double x, double y){
        if(Math.min(Math.abs(x),Math.abs(y)) == Math.abs(x)) return x;
        return y;
    }

    public static double toRobotDegrees(double x){
        return (x % 360 + 360) % 360;
    }

    public Point getBestPoint(double m, double b, Point robot, Point target){
        double xR = robot.x;
        double yR = robot.y;

        if(Math.hypot(target.x - xR, target.y - yR) < Config.targetR) return target;

        double xP = (xR + m * yR - m * b) / (m * m + 1);
        double yP = m * xP + b;
        double d = Math.hypot(xP - xR, yP - yR);

        double x1 = xP + Math.cos(Math.atan(m))
                * (Config.targetT * d + Config.targetR);
        double y1 = m * x1 + b;

        double x2 = xP - Math.cos(Math.atan(m))
                * (Config.targetT * d + Config.targetR);
        double y2 = m * x2 + b;

        double d1 = Math.hypot(target.x - x1, target.y - y1);
        double d2 = Math.hypot(target.x - x2, target.y - y2);

        double dMin = Math.min(d1,d2);

        if(dMin == d1) return new Point(x1,y1);
        return new Point(x2,y2);
    }

    public KodiPursuit execute(){
        pursuitThread = new Thread(() -> {
            lastPoint = waypoints.get(0);
            for(int i=1;!pursuitThread.isInterrupted() && i < waypoints.size();i++){
                targetPoint = waypoints.get(i);

                double dX = targetPoint.x - lastPoint.x;
                double dY = targetPoint.y - lastPoint.y;

                if(dX == 0) dX = 1e-9;

                double m = dY/dX;
                double b = lastPoint.y - m * lastPoint.x;

                SpeedController scH = new SpeedController(
                        Config.hA,
                        Config.hTan,
                        Config.hV
                );

                SpeedController scV = new SpeedController(
                        Config.vA,
                        Config.vTan,
                        Config.vV
                );

                SpeedController scR = new SpeedController(
                        Config.rA,
                        Config.rTan,
                        Config.rV
                );

                boolean check = false;

                double lastDistance = 2e9;

                double targetTheta = loc.theta;

                while (!pursuitThread.isInterrupted() && !check){

                    Point target = getBestPoint(m,b,new Point(loc.x, loc.y),targetPoint);

                    double errorX = target.x - loc.x;
                    double errorY = target.y - loc.y;
                    double errorTheta = targetTheta - loc.theta;

                    errorTheta = minAbs(errorTheta, errorTheta - Math.signum(errorTheta) * 360);

                    double distance = Math.hypot(errorX,errorY);

                    double currentTheta = Math.toRadians(loc.theta);

                    double adjustedX = errorX * Math.cos(currentTheta) + errorY * Math.sin(currentTheta);
                    double adjustedY = -errorX * Math.sin(currentTheta) + errorY * Math.cos(currentTheta);

                    double x = scH.getSpeed(adjustedX);
                    double y = scV.getSpeed(adjustedY);
                    double r = scR.getSpeed(errorTheta);

                    drive.driveRobotCentric(x,y,-r);

                    telemetry.addData("errX",errorX);
                    telemetry.addData("errY",errorY);
                    telemetry.addData("errT",errorTheta);
                    telemetry.addLine();
                    telemetry.addData("x",x);
                    telemetry.addData("y",y);
                    telemetry.addData("r",r);
                    telemetry.update();


                    if(i == waypoints.size() - 1 || !Double.isNaN(targetPoint.theta)){
                        if(distance < Config.toleranceXY
                                && Math.abs(lastDistance-distance) < Config.minimumRate){
                            check = true;
                        }
                    }
                    else{
                        if(distance < Config.targetR * Config.alphaR
                                && Math.abs(lastDistance-distance) < Config.minimumRate){
                            check = true;
                        }
                    }

                    lastDistance = distance;

                }

                scR = new SpeedController(
                        Config.rA,
                        Config.rTan,
                        Config.rV
                );

                double lastError = 2e9;

                while(!pursuitThread.isInterrupted() && !Double.isNaN(targetPoint.theta)){
                    double error = targetPoint.theta - loc.theta;

                    error = minAbs(error, error - Math.signum(error) * 360);

                    double r = scR.getSpeed(error);

                    drive.driveRobotCentric(0,0,-r);

                    telemetry.addData("err",error);
                    telemetry.addLine();
                    telemetry.addData("r",r);
                    telemetry.update();

                    if(Math.abs(error) < 1 && Math.abs(lastError-error) < Config.minimumRate){
                        targetPoint.theta = Double.NaN;
                    }

                    lastError = error;

                }

                lastPoint = targetPoint;
            }
            drive.driveRobotCentric(0,0,0);
        });
        pursuitThread.start();
        return this;
    }

    public boolean finished(){
        return !pursuitThread.isAlive();
    }

    public void kill(){
        loc.stop();
        pursuitThread.interrupt();
        drive.driveRobotCentric(0,0,0);
    }
}
