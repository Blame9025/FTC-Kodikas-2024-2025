package org.firstinspires.ftc.teamcode.Utils;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Drive {
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;

    public Drive(DcMotor frontLeftDrive, DcMotor backLeftDrive, DcMotor frontRightDrive, DcMotor backRightDrive) {
        this.frontLeftDrive = frontLeftDrive;
        this.backLeftDrive = backLeftDrive;
        this.frontRightDrive = frontRightDrive;
        this.backRightDrive = backRightDrive;
    }

    public void robotControl(double x, double y, double r) {
        double fL = y + x + r;
        double bL = y - x + r;
        double fR = y - x - r;
        double bR = y + x - r;

        double maxPower = Math.max(
                Math.max(Math.abs(fL), Math.abs(bL)),
                Math.max(Math.abs(fR), Math.abs(bR))
        );
        maxPower = Math.max(maxPower, 1);


        fL /= maxPower;
        bL /= maxPower;
        fR /= maxPower;
        bR /= maxPower;

        frontLeftDrive.setPower(fL);
        backLeftDrive.setPower(bL);
        frontRightDrive.setPower(fR);
        backRightDrive.setPower(bR);
    }
}
