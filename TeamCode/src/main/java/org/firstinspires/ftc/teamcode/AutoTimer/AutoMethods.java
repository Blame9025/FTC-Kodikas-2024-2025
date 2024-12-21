package org.firstinspires.ftc.teamcode.AutoTimer;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;

public class AutoMethods extends LinearOpMode {

    Telemetry telemetry;

    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    private static final double power = 1;

    Timing.Timer delay = new Timing.Timer(200, TimeUnit.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftRearMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightRearMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");

        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void forward(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(power);
        }
        stopMotors();
    }

    public void backward(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(-power);
        }
        stopMotors();
    }

    public void goRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-power);
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
        }
        stopMotors();

    }

    public void goLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
        }
        stopMotors();
    }

    public void diagonalForwardRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(power);
        }
        stopMotors();
    }

    public void diagonalForwardLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(power);
            backLeftMotor.setPower(power);
        }
        stopMotors();
    }

    public void diagonalBackwardRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-power);
            backLeftMotor.setPower(-power);
        }
        stopMotors();
    }

    public void diagonalBackwardLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(-power);
        }
        stopMotors();
    }

    public void clockwiseLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-power);
            frontLeftMotor.setPower(power);
            backRightMotor.setPower(-power);
            backLeftMotor.setPower(power);
        }
        stopMotors();
    }

    public void clockwiseRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(power);
            frontLeftMotor.setPower(-power);
            backRightMotor.setPower(power);
            backLeftMotor.setPower(-power);
        }

        stopMotors();
    }

    public void stopMotors() {
        frontRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        backLeftMotor.setPower(0);
    }


}
