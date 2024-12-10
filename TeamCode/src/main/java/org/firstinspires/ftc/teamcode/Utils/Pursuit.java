package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.Utils.Waypoint;
import org.firstinspires.ftc.teamcode.Utils.Odometry;
import java.util.List;


public class Pursuit {

    Odometry odometry;
    Drive drive;
    public Pursuit(Drive drive, GamepadEx gamepad, IMU imu) {
        this.drive = drive;
        this.odometry = new Odometry(drive, gamepad, imu);
    }
    //private boolean isWithinRange(Pose2d currentPose, Pose2d targetPose) {
    //    double distance = Math.hypot(
    //            targetPose.getX() - currentPose.getX(),
    //            targetPose.getY() - currentPose.getY()
    //    );
    //    return distance < Config.ACTION_RADIUS;
    //}
    //private Waypoint findLookAheadWaypoint(List<Waypoint> path, Pose2d currentPose) {
    //    for (Waypoint waypoint : path) {
    //        double distance = Math.hypot(
    //                waypoint.getPosition().getX() - currentPose.getX(),
    //                waypoint.getPosition().getY() - currentPose.getY()
    //        );
    //        if (distance > Config.LOOK_AHEAD_DISTANCE) {
    //            return waypoint;
    //        }
    //    }
    //    return path.get(0);
    //}
    //public void followPathWithActions(List<Waypoint> path) {
    //    while (!path.isEmpty()) {
//
    //        Waypoint targetWaypoint = findLookAheadWaypoint(path,   currentPose);
//
    //        if (isWithinRange(currentPose, targetWaypoint.getPosition())) {
    //            if (targetWaypoint.getAction() != null) {
    //                targetWaypoint.getAction().run();
    //            }
    //            path.remove(targetWaypoint);
    //        }
//
    //        Vector2d movementVector = new Vector2d(
    //                targetWaypoint.getPosition().getX() - currentPose.getX(),
    //                targetWaypoint.getPosition().getY() - currentPose.getY()
    //        );
//
    //        drive.robotControl(movementVector.getX(), movementVector.getY(), targetWaypoint.getPosition().getHeading());
    //    }
    //}

}
