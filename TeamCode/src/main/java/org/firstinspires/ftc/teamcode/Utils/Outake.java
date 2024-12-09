package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Outake {

    private DcMotor motorOuttake1,motorOuttake2;
    private boolean alreadyInActionOuttake = false;
    private Position currentPosition = Position.DEFAULT;

    public enum Position {
        DEFAULT(0),
        UPFORINTAKE(50),
        EXTENDED(250);

        public final int val;

        Position(int val) {
            this.val = val;
        }
    }

    public Outake(DcMotor definedIntakeMotor1, DcMotor definedIntakeMotor2) {
        this.motorOuttake1 = definedIntakeMotor1;
        this.motorOuttake2 = definedIntakeMotor2;

        motorOuttake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOuttake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorOuttake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOuttake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorOuttake1.setTargetPosition(Intake.Position.DEFAULT.val);
        motorOuttake2.setTargetPosition(Intake.Position.DEFAULT.val);
    }

    public void setPosition(Position target) {

        if(alreadyInActionOuttake){
            return;
        }
        alreadyInActionOuttake = true;

        motorOuttake1.setTargetPosition(target.val);
        motorOuttake2.setTargetPosition(target.val);

        motorOuttake1.setPower(0.5);
        motorOuttake2.setPower(0.5);

        long startTime = System.currentTimeMillis();
        while(motorOuttake1.isBusy() && motorOuttake2.isBusy()){
            if(System.currentTimeMillis() - startTime > 5000){
                break;
            }
        }
        currentPosition = target;

        alreadyInActionOuttake = true;

    }
    public Position getCurrentPosition() {
        return currentPosition; // Return the stored position
    }


}
