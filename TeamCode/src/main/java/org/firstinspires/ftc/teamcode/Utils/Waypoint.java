package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Pose2d;

public class Waypoint {
    private Pose2d position;
    private Runnable action;

    public Waypoint(Pose2d position, Runnable action) {
        this.position = position;
        this.action = action;
    }

    public Pose2d getPosition() {
        return position;
    }

    public Runnable getAction() {
        return action;
    }
}
