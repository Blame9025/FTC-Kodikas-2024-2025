package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class OuttakeLift {

    private Servo servoArmGrabber , servoGrabber;//de adaugat dupa ce e pus in fisierul KodikasRobot
    private boolean alreadyInAction = false;
    private OuttakeLift.Position currentPositionForOuttake = OuttakeLift.Position.DEFAULT;
    private OuttakeLift.Position currentPositionForGrabber = OuttakeLift.Position.DEFAULT;
    private OuttakeLift.Position currentPositionForArmGrabber = OuttakeLift.Position.DEFAULT;
    KodikasRobot robot;
    Timing.Timer delay = new Timing.Timer(200, TimeUnit.MILLISECONDS);

    public enum Position {
        DEFAULT(0),
        UP(50), // pentru a avea loc sa treaca intake ul
        EXTRACT(200); // pana la cos

        public final double val;

        Position(double val) {
            this.val = val;
        }
    }

    public enum PositionArmGrabber {
        DEFAULT(0),
        UP(0.5);// gheara robotului se ridica ca sa lase game elemntul din gheara

        public final double val;

        PositionArmGrabber(double val) {
            this.val = val;
        }
    }

    public enum PositionGrabber {
        CLOSE(0), // gheara prinde game elementul in clesti
        OPEN(0.5);// gheara robotului in pozitie onitiala cum va sta mereu

        public final double val;

        PositionGrabber(double val) {
            this.val = val;
        }
    }

    public OuttakeLift(KodikasRobot robot, Servo servoGrabber, Servo servoArmGrabber) {
        this.servoGrabber = servoGrabber; // prinde/lasa game element
        this.servoArmGrabber = servoArmGrabber; // ridica bratul cu gheara
        this.robot = robot;

    }

    public void setPositionForGrabber(OuttakeLift.Position target) { // pentru
        if (alreadyInAction) return;

        alreadyInAction = true;
        servoGrabber.setPosition(target.val);

        delay.start();
        while (!delay.done()) {}

        currentPositionForGrabber = target; // Update the current position
        alreadyInAction = false;
    }

}
