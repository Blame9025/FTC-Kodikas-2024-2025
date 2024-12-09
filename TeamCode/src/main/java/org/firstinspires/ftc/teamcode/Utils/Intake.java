package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utils.IntakeLift;

public class Intake {
    private DcMotor intakeMotor;

    private boolean alreadyInAction = false;
    IntakeLift intakeLift;

    public enum Position {
        DEFAULT(0),
        EXTENDED(170);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    public Intake(DcMotor definedIntakeMotor) {
        this.intakeMotor = definedIntakeMotor;
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setTargetPosition(Position.DEFAULT.val);
    }

    public void setPosition(Position target) {
        if (alreadyInAction) return;

        alreadyInAction = true;

        intakeMotor.setTargetPosition(target.val);
        intakeMotor.setPower(0.5);

        long startTime = System.currentTimeMillis();
        while (intakeMotor.isBusy()) {
            if (System.currentTimeMillis() - startTime > 5000) {
                break;
            }
        }
        intakeMotor.setPower(0);
        alreadyInAction = false;
    }

    public void extendIntake() {
        //if()
        if(intakeLift.getCurrentPosition() == IntakeLift.Position.DEFAULT)
            intakeLift.prepareIntakeLift();

        setPosition(Position.EXTENDED);

        intakeLift.extractIntakeLift();

    }

    public void retractIntake() {

        intakeLift.prepareIntakeLift();

        setPosition(Position.DEFAULT);

        intakeLift.retractIntakeLift();
    }
}
