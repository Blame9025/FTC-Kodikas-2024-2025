package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

public class TeleOpTest extends LinearOpMode {


    DcMotor frontLeftMotor; // roata fata dreapta
    DcMotor backLeftMotor; // roata spate dreapta
    DcMotor frontRightMotor; // roata fata dreapta
    DcMotor backRightMotor; // roata spate dreapta
    DcMotor motorIntake; // motorul care extinde glisiera de intake
    DcMotor motorOutake1; /// motor glisiera outtake 2
    DcMotor motorOutake2; // motor glisiera outtake 2
    Servo servoIntake1,servoIntake2; // lasa intake-ul sau urca intakeul (partea verde)
    DcMotor coreHexIntake; // motorul pentru periile de la intake
    Servo servoGrabber, // gheara cu care apuca elementul outtake-ul
            servoArmGrabber; // ridica gheara

    IMU imu;

    boolean active = false; // Starea glisierei (extins/retras)
    boolean activeIntakePull = false; // Stare intake pull (aprins/inchis)
    boolean activeIntakePush = false; // Stare intake push (aprins/inchis)
    boolean activeOutakeLiftForIntake = false;
    boolean activeGrabber = false; // start grabber (apuca/lasa)
    boolean activeArmGrabberUp = false;
    boolean activeBasketPosition = false;
    boolean activeInitialBasketPosition = false;
    boolean intakeStop = true;
    Timing.Timer delay = new Timing.Timer(200, TimeUnit.MILLISECONDS); // Timer pentru debounce
    Timing.Timer debounceTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceA
    Timing.Timer debounceTimerB = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceB
    Timing.Timer debounceTimerX = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceX
    Timing.Timer debounceTimerY = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceY
    Timing.Timer debounceTimerRT = new Timing.Timer(500, TimeUnit.MILLISECONDS); // Timer pentru debounceRT
    Timing.Timer debounceTimerArrow1 = new Timing.Timer(500, TimeUnit.MILLISECONDS); //
    Timing.Timer debounceTimerArrow3 = new Timing.Timer(500, TimeUnit.MILLISECONDS); //
    Timing.Timer outakeTimer = new Timing.Timer(1000,TimeUnit.MILLISECONDS); //TImer pentru ridicare outake-ului inaite sa plece intake-ul
    final int forwardPosition = 170; // Poziția extinsă
    final int backwardPosition = 0; // Poziția retrasă
    final int positionOuttakeUpForIntake = 50; // Pozitia pana la care se ridica outtake ul ca s aiba loc glisiera de la intake sa se deschida
    final int basketPosition = 200; // pozitie outtake pentru cosul de sus
    final int initialOuttakePosition = 0;
    final double powerForCorehex = 0.5;
    final double power = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(
                new Motor(hardwareMap, "leftFrontMotor"),
                new Motor(hardwareMap, "rightFrontMotor"),
                new Motor(hardwareMap, "leftRearMotor"),
                new Motor(hardwareMap, "rightRearMotor")
        );

        // Inițializarea hardware-ului pentru motoare
        frontLeftMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        backLeftMotor = hardwareMap.dcMotor.get("leftRearMotor");
        frontRightMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        backRightMotor = hardwareMap.dcMotor.get("rightRearMotor");
        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorOutake1 = hardwareMap.dcMotor.get("motorOutake1");
        motorOutake2 = hardwareMap.dcMotor.get("motorOutake2");
        coreHexIntake = hardwareMap.dcMotor.get("coreHexIntake");

        // Inițializarea hardware-ului pentru servo
        servoIntake1 = hardwareMap.servo.get("servoIntake1");
        servoIntake2 = hardwareMap.servo.get("servoIntake2");
        servoGrabber = hardwareMap.servo.get("servoGrabber"); // gheara cu care apuca elementul outtake-ul
        servoArmGrabber = hardwareMap.servo.get("servoArmGrabber"); // ridica gheara

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        servoIntake1.setDirection(Servo.Direction.REVERSE); // de la stanga la dreapta cum te uiti spre intake


        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorOutake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        coreHexIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        coreHexIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // Pornește timer-ul de debounce
        debounceTimer.start();
        debounceTimerB.start();
        debounceTimerX.start();
        debounceTimerY.start();
        debounceTimerRT.start();
        debounceTimerArrow1.start();
        debounceTimerArrow3.start();
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart(); // Așteaptă apăsarea butonului "Start" din Driver Station

        // Bucla principală
        while (opModeIsActive()) {
            double lt = gamepad1.left_trigger; // Actualizează constant valorile lui lt și rt
            double rt = gamepad1.right_trigger;

            drive.driveRobotCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX(),
                    false
            );

            intakeMove(lt, rt); // Control continuu pentru intake / BUTTON 'RT' AND 'LT'
            armGrabberMove(); // Daca apesi RB bratul de pe outtake se va ridica pentru a lasa piesa in cos / BUTTON 'RB'

            handleGlisiera();  // Extinde glisisera pana la pozitia maxima si la a 2 a apasare se inchide pana la pozitia 0 / BUTTON 'A'
            runIntake(powerForCorehex); // Control continuu pentru periile de pe intake la tragerea elementelor din intake (pull) / BUTTON 'X'
            reverseIntake(); // Control continuu pentru periile de pe intake la impingerea elementelor din intake (push) / BUTTON 'B'
            outtakeGrabber(); // Daca apesi Y prinde piesa iar la a 2 apasare il lasa jos / BUTTON 'Y'
            arrowMove1(); // Cand apas dpadUp se va ridica pana la pozitia de cos, dar daca vreau sa l las jos trebuie sa apas pe dpadDown
            arrowMove3(); // Cand apas dpadDown prima oara se lasa pana la pozitia la care se poate extinde glisera de intake , iar a doua oara se duce pana la pozitia 0

            //START BOOLEAN SECTION

            if(motorIntake.getCurrentPosition() >= forwardPosition - 10){
                active = true;
            } else{
                active = false;
            }

            /*if(coreHexIntake.getPower() > 0){
                activeIntakePush = true;
                intakeStop = false;
            } else if(coreHexIntake.getPower() == 0){
                activeIntakePush = false;
                activeIntakePull = false;
                intakeStop = true;
            } else if(!intakeStop && !activeIntakePush){
                activeIntakePull = true; // posibil e gresit pentru ca puterea motorului va fi data tot cu 1 nu cu -1
            }*/

            if((motorOutake1.getCurrentPosition() >= positionOuttakeUpForIntake - 5 && motorOutake2.getCurrentPosition() >= positionOuttakeUpForIntake - 5) || (motorOutake1.getCurrentPosition() <= positionOuttakeUpForIntake + 5 && motorOutake2.getCurrentPosition() <= positionOuttakeUpForIntake + 5)){
                activeOutakeLiftForIntake = true;
            } else if((motorOutake1.getCurrentPosition() < positionOuttakeUpForIntake - 5 && motorOutake2.getCurrentPosition() < positionOuttakeUpForIntake - 5) || (motorOutake1.getCurrentPosition() > positionOuttakeUpForIntake + 5 && motorOutake2.getCurrentPosition() > positionOuttakeUpForIntake + 5)){
                activeOutakeLiftForIntake = false;
            }

            if(servoGrabber.getPosition() > 0){
                activeGrabber = true;
            } else{
                activeGrabber = false;
            }

            if(servoArmGrabber.getPosition() > 0){
                activeArmGrabberUp = true;
            } else{
                activeArmGrabberUp = false;
            }

            if(motorOutake1.getCurrentPosition() >= basketPosition-5 && motorOutake2.getCurrentPosition() >= basketPosition-5){ // (195)
                activeBasketPosition = true;
            } else if(motorOutake1.getCurrentPosition() < basketPosition-5 && motorOutake2.getCurrentPosition() < basketPosition-5){
                activeBasketPosition = false;
            }

            if(motorOutake1.getCurrentPosition() <= 5 && motorOutake2.getCurrentPosition() <= 5){ // (45,55)
                activeInitialBasketPosition = true;
            } else{
                activeInitialBasketPosition = false;
            }

            //END BOOLEAN SECTION

            telemetry.addData("Glisiera Intake Active: ", active);
            telemetry.addData("Outake Lift For Intake: ",activeOutakeLiftForIntake);
            telemetry.addData("Intake Pull: ",activeIntakePull);
            telemetry.addData("Intake Push: ",activeIntakePush);
            telemetry.addData("Grabber Status: ", activeGrabber);
            telemetry.addData("Arm Grabber Up: ", activeArmGrabberUp);
            telemetry.update();
        }
    }

    //START SECTION BUTTON 'RT' AND 'LB'

    public void intakeMove(double lt, double rt) {  // daca apas rt intake-ul se va duce in fata, iar daca apas lt se va duce in spate
        int position = motorIntake.getCurrentPosition();

        if (position <= 1600 && (rt > 0 || lt > 0) && motorOutake1.getCurrentPosition() >= positionOuttakeUpForIntake) {
            motorIntake.setPower(rt - lt);
        } else if (position >= 1600 && motorOutake1.getCurrentPosition() >= positionOuttakeUpForIntake) {
            motorIntake.setPower(-1);
        }else if (position < 0 || position > 1600) {
            motorIntake.setPower(0); // Protecție la limitele pozițiilor
        }else {
            motorIntake.setPower(0); // Oprește motorul dacă nu există comenzi valide
        }
    }

    //END SECTION BUTTON 'RT' AND 'LB'

    // START SECTION BUTTON 'A'

    public void handleGlisiera() {

        // Verifică dacă butonul `a` a fost apăsat și timer-ul de debounce a expirat
        if (gamepad1.a && debounceTimer.done()) {

            debounceTimer.start(); // Repornește timer-ul pentru următorul ciclu
            if(!active){

                liftOutakeForIntake();
                if(motorOutake1.getCurrentPosition() >= positionOuttakeUpForIntake-5){
                    moveToPosition(forwardPosition);
                    active = true;
                }
            } else if(active) {
                moveToPosition(backwardPosition);
                if(motorIntake.getCurrentPosition() <= 20) {
                    downOutakeForIntake();
                    active = false;
                }

            }

        }
    }

    public void moveToPosition(int targetPosition) {

        motorIntake.setTargetPosition(targetPosition);
        motorIntake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorIntake.setPower(power);

        servoIntake1.setPosition(0);
        servoIntake2.setPosition(0);

        while (motorIntake.isBusy() && opModeIsActive()) {

            if (motorIntake.getCurrentPosition() >= 80){
                servoIntake1.setPosition(1);
                servoIntake2.setPosition(1);
            } else if (motorIntake.getCurrentPosition() < 80){
                servoIntake1.setPosition(0.5);
                servoIntake2.setPosition(0.5);
            }

            telemetry.addData("Motor Position", motorIntake.getCurrentPosition());
            telemetry.addData("Target Position", motorIntake.getTargetPosition());
            telemetry.update();
        }

        motorIntake.setPower(0); // Oprește motorul după atingerea poziției
        motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Target Position", "Done!");
        telemetry.update();
    }

    public void liftOutakeForIntake(){

        telemetry.addData("Status Outake: ", "Lifting outtake for intake!");
        telemetry.update();

        motorOutake1.setTargetPosition(800);
        motorOutake2.setTargetPosition(800);

        motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorOutake1.setPower(power);
        motorOutake2.setPower(power);

        while (motorOutake1.isBusy() && motorOutake2.isBusy() && opModeIsActive()) {
            telemetry.addData("Outtake Position", motorOutake1.getCurrentPosition());
            telemetry.update();
        }

        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outakeTimer.start();
        while (!outakeTimer.done() && opModeIsActive()) {
            telemetry.addData("Waiting Timer", outakeTimer.remainingTime());
            telemetry.update();
        }

        activeOutakeLiftForIntake = true;

    }

    public void downOutakeForIntake(){

        telemetry.addData("Status Outake: ", "Lowering outtake for intake!");
        telemetry.update();

        motorOutake1.setTargetPosition(0);
        motorOutake2.setTargetPosition(0);

        motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorOutake1.setPower(power);
        motorOutake2.setPower(power);

        while (motorOutake1.isBusy() && motorOutake2.isBusy() && opModeIsActive()) {
            telemetry.addData("Outtake Position", motorOutake1.getCurrentPosition());
            telemetry.update();
        }

        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        activeOutakeLiftForIntake = false;

    }

    // END SECTION BUTTON 'A'

    //START SECTION BUTTON 'X' // trage intake | pull = trage

    public void runIntake(double power){

        if (gamepad1.x && debounceTimerX.done()) {

            debounceTimerX.start();
            if (!activeIntakePull && !activeIntakePush) {

                coreHexIntake.setPower(power);
                activeIntakePull = true;

            } else if (activeIntakePull && !activeIntakePush) {

                coreHexIntake.setPower(0);
                activeIntakePull = false;
            }
        }

    }

    //END SECTION BUTTON 'X'

    //START SECTION BUTTON 'B' // scuipa intake (HAWK TUAH) | push = impinge = scuipa

    public void reverseIntake(){

        if (gamepad1.b && debounceTimerB.done()) {

            debounceTimerB.start();
            if (!activeIntakePush && !activeIntakePull) {

                coreHexIntake.setPower(-power);
                activeIntakePush = true;
                intakeStop = false;

            } else if(activeIntakePush && !activeIntakePull){

                coreHexIntake.setPower(0);
                activeIntakePush = false;
                intakeStop = true;
            }
        }

    }

    //END SECTION BUTTON 'B'

    // START SECTION BUTTON 'Y'

    public void outtakeGrabber(){

        if(gamepad1.y && debounceTimerY.done()){

            debounceTimerY.start();
            if (!activeGrabber) {

                servoGrabber.setPosition(0);
                activeGrabber = true;

            } else {

                servoGrabber.setPosition(0.5);
                activeGrabber = false;
            }

        }

    }

    //END SECTION BUTTON 'Y'

    // START SECTION BUTTON 'RB'

    public void armGrabberMove(){

        if(gamepad1.right_bumper && debounceTimerRT.done()) {

            debounceTimerRT.start();
            if (!activeArmGrabberUp) {

                servoArmGrabber.setPosition(0.5);
                activeArmGrabberUp = true;
            } else {
                servoArmGrabber.setPosition(0);
                activeArmGrabberUp = false;
            }
        }

    }

    //END SECTION BUTTON 'RB'

    //START SECTION ARROW '1'

    public void arrowMove1(){

        if(gamepad1.dpad_up && debounceTimerArrow1.done()){ // daca apas prima data a glisiera de outtake se duce sus daca apas a doua oara se duce la pozitia astfel incat glisiera de intake sa aibe loc sa se extinda, iar daca apas a treia oara glisiera se lasa in pozitia 0 astfel sa poata sa prinda game elementul

            debounceTimerArrow1.start();
            if(!activeBasketPosition && debounceTimerArrow1.done()) {

                moveToPositionOuttake(basketPosition);
                activeBasketPosition = true;


                //moveToPositionOuttake(initialOuttakePosition); cand apas a 2 a oara nu se intampla nimic
                // pentru ca daca vreau sa l las jos trebuie sa apas pe arrow 3

            }
        }

    }

    public void moveToPositionOuttake(int targetPositionForOuttake){

        motorOutake1.setTargetPosition(targetPositionForOuttake);
        motorOutake2.setTargetPosition(targetPositionForOuttake);

        motorOutake1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorOutake2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorOutake1.setPower(power);
        motorOutake2.setPower(power);

        while (motorOutake1.isBusy() && motorOutake2.isBusy() && opModeIsActive()) {
            telemetry.addData("Outtake Position", motorOutake1.getCurrentPosition());
            telemetry.update();
        }

        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorOutake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorOutake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }

    //END SECTION ARROW '1'

    //START SECTION ARROW '3'

    public void arrowMove3(){


        if(gamepad1.dpad_down && debounceTimerArrow3.done()){

            debounceTimerArrow3.start();
            if(!activeInitialBasketPosition && debounceTimerArrow3.done() && motorOutake1.getCurrentPosition() > positionOuttakeUpForIntake-5){

                moveToPositionOuttake(positionOuttakeUpForIntake);
                activeInitialBasketPosition = false;
                activeOutakeLiftForIntake = true;

            } else if(!activeInitialBasketPosition && debounceTimerArrow3.done() && motorOutake1.getCurrentPosition() <= positionOuttakeUpForIntake+5){

                moveToPositionOuttake(initialOuttakePosition);
                activeInitialBasketPosition = true;
                activeOutakeLiftForIntake = false;

            }

        }

    }

    //END SECTION ARROW '3'

    public void stopAllMotor() {
        motorOutake1.setPower(0);
        motorOutake2.setPower(0);
        motorIntake.setPower(0);

        servoIntake1.setPosition(0);
        servoIntake2.setPosition(0);
        servoArmGrabber.setPosition(0);
        servoGrabber.setPosition(0);
    }

}
