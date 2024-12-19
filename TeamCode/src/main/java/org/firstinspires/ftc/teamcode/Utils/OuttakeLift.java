package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class OuttakeLift {

    private Servo servoArmGrabber , servoGrabber;
    private OuttakeLift.PositionGrabber currentPositionForGrabber = OuttakeLift.PositionGrabber.CLOSE;
    private OuttakeLift.PositionArmGrabber currentPositionForArmGrabber = OuttakeLift.PositionArmGrabber.DEFAULT;
    KodikasRobot robot;

    public enum PositionArmGrabber {
        DEFAULT(0),
        UP(0.4),// gheara robotului se ridica ca sa lase game elemntul din gheara
        IDLE(0.8);
        public final double val;

        PositionArmGrabber(double val) {
            this.val = val;
        }
    }

    public enum PositionGrabber {
        CLOSE(0), // gheara prinde game elementul in clesti
        OPEN(0.2);// gheara robotului in pozitie onitiala cum va sta mereu

        public final double val;

        PositionGrabber(double val) {
            this.val = val;
        }
    }

    public OuttakeLift(KodikasRobot robot, Servo servoGrabber, Servo servoArmGrabber) {
        this.servoGrabber = servoGrabber; // prinde lasa game element
        this.servoArmGrabber = servoArmGrabber; // ridica bratul cu gheara
        this.robot = robot;

    }

    public void setPositionForGrabber(PositionGrabber target) { // pentru gheara care prinde game elementul
        servoGrabber.setPosition(target.val);

        currentPositionForGrabber = target; // Update the current position
    }

    public void setPositionForArmGrabber(PositionArmGrabber target) { // pentru gheara care prinde game elementul
        servoArmGrabber.setPosition(target.val);

        currentPositionForArmGrabber = target; // Update the current position
    }

    public void closeGrabber(){
        setPositionForGrabber(PositionGrabber.CLOSE);

    }

    public void openGrabber(){
        setPositionForGrabber(PositionGrabber.OPEN);

    }

    public void downArmGrabber(){

        setPositionForArmGrabber(PositionArmGrabber.DEFAULT);

    }
    public void idleArmGrabber(){

        setPositionForArmGrabber(PositionArmGrabber.IDLE);

    }
    public void upArmGrabber(){

        setPositionForArmGrabber(PositionArmGrabber.UP);

    }

}
