package org.firstinspires.ftc.teamcode.Utils;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class OuttakeLift {

    private Servo servoArmGrabber , servoGrabber;//de adaugat dupa ce e pus in fisierul KodikasRobot
    private boolean alreadyInActionGrabber = false;
    private boolean alreadyInActionArmGrabber = false;
    private OuttakeLift.PositionGrabber currentPositionForGrabber = OuttakeLift.PositionGrabber.CLOSE;
    private OuttakeLift.PositionArmGrabber currentPositionForArmGrabber = OuttakeLift.PositionArmGrabber.DEFAULT;
    KodikasRobot robot;
    Timing.Timer delayForGrabber = new Timing.Timer(2000, TimeUnit.MILLISECONDS);
    Timing.Timer delayForArmGrabber = new Timing.Timer(2000, TimeUnit.MILLISECONDS);

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
        this.servoGrabber = servoGrabber; // prinde lasa game element
        this.servoArmGrabber = servoArmGrabber; // ridica bratul cu gheara
        this.robot = robot;

    }

    public void setPositionForGrabber(PositionGrabber target) { // pentru gheara care prinde game elementul
        if (alreadyInActionGrabber) return;
        if(servoGrabber.getPosition() == target.val) return;

        alreadyInActionGrabber = true;
        servoGrabber.setPosition(target.val);

        delayForGrabber.start();
        while (!delayForGrabber.done()) {
            telemetry.addData("Grabber: ", alreadyInActionGrabber);
            telemetry.update();

        }

        currentPositionForGrabber = target; // Update the current position
        alreadyInActionGrabber = false;
    }

    public void setPositionForArmGrabber(PositionArmGrabber target) { // pentru gheara care prinde game elementul
        if (alreadyInActionArmGrabber) return;
        if(servoArmGrabber.getPosition() == target.val) return;

        alreadyInActionArmGrabber = true;
        servoArmGrabber.setPosition(target.val);

        delayForArmGrabber.start();
        while (!delayForArmGrabber.done()) {
            telemetry.addData("Grabber: ", alreadyInActionArmGrabber);
            telemetry.update();

        }

        currentPositionForArmGrabber = target; // Update the current position
        alreadyInActionArmGrabber = false;
    }

    public void closeGrabber(){
        if(currentPositionForGrabber == PositionGrabber.CLOSE){
            return;
        }else{
            setPositionForGrabber(PositionGrabber.CLOSE);
        }

    }

    public void openGrabber(){
        if(currentPositionForGrabber == PositionGrabber.OPEN){
            return;
        }else{
            setPositionForGrabber(PositionGrabber.OPEN);
        }
    }

    public void downArmGrabber(){
        if(currentPositionForArmGrabber == PositionArmGrabber.DEFAULT){
            return;
        } else{
            setPositionForArmGrabber(PositionArmGrabber.DEFAULT);
        }
    }

    public void upArmGrabber(){
        if(currentPositionForArmGrabber == PositionArmGrabber.UP){
            return;
        } else{
            setPositionForArmGrabber(PositionArmGrabber.UP);
        }
    }

}
