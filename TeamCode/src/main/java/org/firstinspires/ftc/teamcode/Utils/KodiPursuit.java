package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class KodiPursuit {

    public KodiLocalization loc;

    Thread pursuitThread;

    public ArrayList<Point> waypoints = new ArrayList<>();

    public Point lastPoint, targetPoint;

    public MecanumDrive drive;

    Telemetry telemetry;

    public KodiPursuit(MecanumDrive drive, Telemetry telemetry, KodiLocalization loc){
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

        double offset = Math.cos(Math.atan(m)) * (Config.targetT * d + Config.targetR);

        double x = xP + Math.signum(target.x - xP) * offset;
        double y = m * x + b;

        return new Point(x,y);
    }

    public KodiPursuit execute(){
        pursuitThread = new Thread(() -> {
            lastPoint = loc.getLocAsPoint();
            double targetTheta = lastPoint.theta;
            SpeedController scH = new SpeedController(
                    Config.hA,
                    Config.hV,
                    Config.hBr
            );

            SpeedController scV = new SpeedController(
                    Config.vA,
                    Config.vV,
                    Config.vBr
            );

            SpeedController scR = new SpeedController(
                    Config.rA,
                    Config.rV,
                    Config.rBr
            );
            for(int i=0;!pursuitThread.isInterrupted() && i < waypoints.size();i++){
                targetPoint = waypoints.get(i);

                double dX = targetPoint.x - lastPoint.x;
                double dY = targetPoint.y - lastPoint.y;

                if(dX == 0) dX = 1e-9;

                double m = dY/dX;
                double b = lastPoint.y - m * lastPoint.x;

                boolean check = false;

                double lastDistance = 2e9;

                while (!pursuitThread.isInterrupted() && !check){

                    scH.updateCoef(Config.hA, Config.hV, Config.hBr);
                    scV.updateCoef(Config.vA, Config.vV, Config.vBr);
                    scR.updateCoef(Config.rA, Config.rV, Config.rBr);

                    Point target = getBestPoint(m,b,loc.getLocAsPoint(),targetPoint);

                    double errorX = target.x - loc.x;
                    double errorY = target.y - loc.y;
                    double errorTheta = targetTheta - loc.theta;

                    errorTheta = minAbs(errorTheta, errorTheta - Math.signum(errorTheta) * 360);

                    double distance = Math.hypot(errorX,errorY);

                    double movementAngle = Math.atan2(errorY,errorX);

                    double currentTheta = Math.toRadians(360 - loc.theta);

                    double adjustedX = distance * Math.cos(movementAngle - currentTheta);
                    double adjustedY = distance * Math.sin(movementAngle - currentTheta);

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
                        if(distance < Config.targetR){
                            check = true;
                        }
                    }

                    lastDistance = distance;

                }

                scR.resetSpeed();

                double lastError = 2e9;

                while(!pursuitThread.isInterrupted() && !Double.isNaN(targetPoint.theta)){

                    scR.updateCoef(Config.rA, Config.rV, Config.rBr);


                    double error = targetPoint.theta - loc.theta;

                    error = minAbs(error, error - Math.signum(error) * 360);

                    double r = scR.getSpeed(error);

                    drive.driveRobotCentric(0,0,-r);

                    telemetry.addData("err",error);
                    telemetry.addLine();
                    telemetry.addData("r",r);
                    telemetry.update();

                    if(Math.abs(error) < Config.toleranceR
                            && Math.abs(lastError-error) < Config.minimumRate){
                        targetTheta = targetPoint.theta;
                        targetPoint.theta = Double.NaN;
                        scH.resetSpeed();
                        scV.resetSpeed();
                        scR.resetSpeed();
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
