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

    Timing.Timer delay = new Timing.Timer(200, TimeUnit.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");

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
            frontRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(1);
            backLeftMotor.setPower(1);
        }
        stopMotors();
    }

    public void backward(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-1);
            frontLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
        }
        stopMotors();
    }

    public void goRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-1);
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(1);
            backLeftMotor.setPower(-1);
        }
        stopMotors();

    }

    public void goLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(1);
            frontLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
        }
        stopMotors();
    }

    public void diagonalForwardRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(1);
        }
        stopMotors();
    }

    public void diagonalForwardLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(1);
            backLeftMotor.setPower(1);
        }
        stopMotors();
    }

    public void diagonalBackwardRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-1);
            backLeftMotor.setPower(-1);
        }
        stopMotors();
    }

    public void diagonalBackwardLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontLeftMotor.setPower(-1);
            backRightMotor.setPower(-1);
        }
        stopMotors();
    }

    public void clockwiseLeft(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(-1);
            frontLeftMotor.setPower(1);
            backRightMotor.setPower(-1);
            backLeftMotor.setPower(1);
        }
        stopMotors();
    }

    public void clockwiseRight(long timer){
        Timing.Timer delay = new Timing.Timer(timer, TimeUnit.MILLISECONDS);
        delay.start();

        while(!delay.done()){
            frontRightMotor.setPower(1);
            frontLeftMotor.setPower(-1);
            backRightMotor.setPower(1);
            backLeftMotor.setPower(-1);
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
