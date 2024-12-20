package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class KodikasRobot {
    Telemetry telemetry;

    Motor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;

    Intake intake;
    IntakeLift intakeLift;
    Outake outake;
    OuttakeLift outtakeLift;

    DcMotor motorIntake;
    Servo servoArm;
    Servo servoArmGrabber;
    DcMotor motorOutake1, motorOutake2;
    DcMotor coreHexIntake;
    Servo servoIntake1, servoIntake2;
    MecanumDrive drive;
    void initHardware(HardwareMap hardwareMap)
    {

        frontLeftMotor = new Motor(hardwareMap, "leftFrontMotor");
        backLeftMotor = new Motor(hardwareMap, "leftRearMotor");
        frontRightMotor = new Motor(hardwareMap, "rightFrontMotor");
        backRightMotor = new Motor(hardwareMap, "rightRearMotor");

        frontLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
        servoArm = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

       // servoIntake1.setDirection(Servo.Direction.FORWARD);
        servoIntake2.setDirection(Servo.Direction.REVERSE);

        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake2.setDirection(DcMotor.Direction.REVERSE);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setDirection(DcMotor.Direction.REVERSE);

    }
    public KodikasRobot(HardwareMap hardwareMap, Telemetry telemetry){
        initHardware(hardwareMap);
        this.intake = new Intake(this,motorIntake,coreHexIntake);
        this.intakeLift = new IntakeLift(this,servoIntake1, servoIntake1);
        this.outake = new Outake(this,motorOutake1, motorOutake2);
        this.outtakeLift = new OuttakeLift(this,servoArm,servoArmGrabber);
        this.telemetry = telemetry;


        this.drive = new MecanumDrive(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor);

    }
    public Telemetry getTelemetry()
    {
        return  telemetry;
    }
    public Intake getIntakeSession(){
        return intake;
    }
    public IntakeLift getIntakeLiftSession(){
        return intakeLift;
    }
    public Outake getOutakeSession(){
        return outake;
    }
    public OuttakeLift getOutakeLiftsession(){
        return outtakeLift;
    }
    public MecanumDrive getDriveSession() { return drive; }
    public DcMotor getCoreHexIntake() { return coreHexIntake; }
    public Motor getFRightMotor(){return frontRightMotor;}
    public Motor getBRightMotor(){return backRightMotor;}

}
