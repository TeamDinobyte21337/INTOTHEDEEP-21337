package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.ExpansionHubPIDFPositionParams;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Dinobyters", group = "Linear OpMode")
public class Dinobyters extends LinearOpMode {

    DcMotorEx FLMotor; // front left motor
    DcMotorEx FRMotor; // front right motor
    DcMotorEx BLMotor; // back left motor
    DcMotorEx BRMotor; // back right motor
    DcMotorEx HRSliderMotor; // right horizontal slider motor
    DcMotorEx HLSliderMotor; // left horizontal motor
    DcMotorEx VRSliderMotor;// right vertical slider motor
    DcMotorEx VLSliderMotor; // left vertical slider motor
    //Servo HClaw; // open and closes horizontal claw
    //Servo H180; // turns horizontal claw 180
    //Servo RightH90;  // move horizontal claw 90 degrees
    //Servo LeftH90; // same as above
    //Servo VClaw;  // open and close vertical claw
    //Servo V180;  // turn vertical claw 180 (up and down)
    //Servo RightV180;  // move vertical claw 90 degrees
    //Servo LeftV180; // same as above 
    // DcMotorEx horizontalWheel;
    //  DcMotorEx verticalWheel;

    boolean openClaw = true; // these boolean allows us to use same button for different commands
    boolean move90 = true; //
    boolean turn180 = true;
    boolean move180 = true;
    boolean fullExtend = true;

    // ticks for arm

    double HRSliderMotorTarget;
    double HLSliderMotorTarget;
    double HRSliderMotorTicks = 2300;
    double HLSliderMotorTicks = 2300;

    double VRSliderMotorTicks = 537.7;
    double VLSliderMotorTicks = 537.7;
    double VRSliderMotorTarget;
    double VLSliderMotorTarget;

    //PID
    double integralSum = 0;
    double Kp = 17; // play with Kp, Ki, Kd until it runs smoothly
    double Ki = 0.15;
    double Kd = 1;

    ElapsedTime timer = new ElapsedTime();
    double lastError = 0;

    //Odometry
    double calibrationSpeed = 0.5;
    double R = 2.4; // radius
    double N = 537.7; // ticks per revolution
    double GEAR_RATIO = 3;
    double TICKS_PER_INCH = R * Math.PI*GEAR_RATIO/N;

            

    public int currentRightPosition = 0;
    public int currentLeftPosition = 0;
    public int currentStrafePosition = 0;

    private final int oldRightPosition = 0;
    private final int oldLeftPosition = 0;
    private final int oldStrafePosition = 0;



    public void runOpMode() throws InterruptedException{

        FLMotor = hardwareMap.get(DcMotorEx.class, "FLMotor");
        FLMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLMotor.setDirection(DcMotorEx.Direction.REVERSE);
        FLMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        FRMotor = hardwareMap.get(DcMotorEx.class, "FRMotor");
        FRMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setDirection(DcMotorEx.Direction.REVERSE);
        FRMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        BLMotor = hardwareMap.get(DcMotorEx.class, "BLMotor");
        BLMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setDirection(DcMotorEx.Direction.FORWARD);
        BLMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        BRMotor = hardwareMap.get(DcMotorEx.class, "BRMotor");
        BRMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setDirection(DcMotorEx.Direction.FORWARD );
        BRMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        HRSliderMotor = hardwareMap.get(DcMotorEx.class, "HRSliderMotor");
        HLSliderMotor =hardwareMap.get(DcMotorEx.class, "HLSliderMotor");
        HRSliderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        HLSliderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //HRSliderMotor.setDirection(DcMotor.Direction.REVERSE);
        //HLSliderMotor.setDirection(DcMotor.Direction.REVERSE);
        HLSliderMotor.setDirection(DcMotorEx.Direction.FORWARD);
        HRSliderMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //HRSliderMotor.setTargetPosition((int)HRSliderMotorTarget);
        //HLSliderMotor.setTargetPosition((int)HLSliderMotorTarget);
        //HRSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //HLSliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // HLSliderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//    //    HRSliderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        VRSliderMotor = hardwareMap.get(DcMotorEx.class, "VRSliderMotor");
        VLSliderMotor = hardwareMap.get(DcMotorEx.class, "VLSliderMotor");
        VRSliderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        VLSliderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        VRSliderMotor.setDirection(DcMotorEx.Direction.FORWARD);
        VLSliderMotor.setDirection(DcMotorEx.Direction.REVERSE);
        //VRSliderMotor.setTargetPosition((int)VRSliderMotorTarget);
        //VLSliderMotor.setTargetPosition((int)VLSliderMotorTarget);
        VRSliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        VLSliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        VLSliderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        VRSliderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        /*
        HClaw = hardwareMap.get(Servo.class, "HClaw");
        H180 =  hardwareMap.get(Servo.class, "H90");
        RightH90 = hardwareMap.get(Servo.class, "RightH90");
        LeftH90 = hardwareMap.get(Servo.class, "LeftH90");

        VClaw = hardwareMap.get(Servo.class, "VClaw");
        V180= hardwareMap.get(Servo.class, "V180");
        RightV180 = hardwareMap.get(Servo.class, "RightH180");
        LeftV180 = hardwareMap.get(Servo.class, "LeftV180");
        */
        //  horizontalWheel = hardwareMap.get(DcMotorEx.class, "horizontalWheel");
        // verticalWheel = hardwareMap.get(DcMotorEx.class, "verticalWheel");

        waitForStart();

        while(opModeIsActive()) {

            double drive = 1.2 * gamepad1.left_stick_y;  // Forward and backward
            double strafe = 1 * gamepad1.left_stick_x;  // Left and right (strafing)
            double turn = gamepad1.right_stick_x; // self explanatory
            // double horizontal = gamepad2.left_stick_y;//// Rotation

            double up = 0.5 * gamepad1.left_trigger; // triggers
            double down = -0.5 * gamepad1.right_trigger;

            FLMotor.setPower(-drive + strafe + turn); // motor directions
            FRMotor.setPower(drive + strafe + turn);
            BLMotor.setPower(drive - strafe - turn);
            BRMotor.setPower(-drive - strafe - turn);

            VRSliderMotor.setPower(gamepad2.right_stick_y);
            VLSliderMotor.setPower(gamepad2.right_stick_y);

            telemetry.addData("Joystick value", gamepad2.left_stick_y);
            telemetry.update();
            HRSliderMotor.setPower(-gamepad2.left_stick_y);


            double power = PIDControl(100,100);
            FLMotor.setPower(power);
            FRMotor.setPower(power);
            BLMotor.setPower(power);
            BRMotor.setPower(power);
            //
            //
            // horizontalWheel.setPower(power);
            //     verticalWheel.setPower(power);



            // horizontal arm

            // if (gamepad2.dpad_right) {
            //maxH(0.5);
            //}

            //if (gamepad2.dpad_up) {
            //  maxH(1.0);
            //}

            //if (gamepad2.dpad_down) {
            //  maxH(0);

            // }


            // horizontal claw
            /*
            if (gamepad2.b & openClaw){
                HClaw.setPosition(0.25);
                openClaw = false;
            }
            else if (gamepad2.b & !openClaw) {
                HClaw.setPosition(0.0);
                openClaw = true;
            }

            if (gamepad2.y & move90 & turn180) {
                RightH90.setPosition(0.5);
                LeftH90.setPosition(0.5);
                sleep(100);
                H180.setPosition(1.0);

                move90 = false;
                turn180 = false;
            }

            else if (gamepad2.y & !move90 & !turn180){
                RightH90.setPosition(0.0);
                LeftH90.setPosition(0.0);
                sleep(100);
                H180.setPosition(0.0);
                move90 = true;
                turn180 = true;
            }

            // vertical claw
            if (gamepad2.x & openClaw){
                VClaw.setPosition(0.25);
                openClaw = false;
            }
            else if (gamepad2.x & !openClaw) {
                VClaw.setPosition(0.0);
                openClaw = true;
            }

            if (gamepad2.a & move180 & turn180) {
                RightV180.setPosition(0.5);
                LeftV180.setPosition(0.5);
                sleep(100);
                H180.setPosition(1.0);

                move180 = false;
                turn180 = false;
            }

            else if (gamepad2.a & !move180 & !turn180){
                RightV180.setPosition(0.0);
                LeftV180.setPosition(0.0);
                sleep(100);
                H180.setPosition(0.0);

                move180 = true;
                turn180 = true;
            }
            */

            // vertical slider extender


            // if (gamepad1.dpad_left & fullExtend){

            // VLSliderMotor.setPower(0.7);
            //VRSliderMotor.setPower(0.5);
            // sleep(100);

            // fullExtend = true;
            // }

            //if (gamepad1.dpad_left & !fullExtend) {
            //  VRSliderMotor.setPower(0.0);
            // VLSliderMotor.setPower(0.0);
            //  sleep(100);

            // fullExtend = false;
            // }


        }

    }
    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki);
    }

    /*
    public void fullH(double turnage) {
        int i = 0;
        while (i < 1) {
            VRSliderMotorTarget = VRSliderMotorTicks * turnage;
            VLSliderMotorTarget = VLSliderMotorTicks * turnage;
            VLSliderMotor.setTargetPosition((int) VLSliderMotorTarget);
            VRSliderMotor.setTargetPosition((int) VRSliderMotorTarget);
            VLSliderMotor.setPower(.2);
            VRSliderMotor.setPower(.2);
            VLSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VRSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            i++;
        }
    }
    */


    public void originH() {
        HRSliderMotor.setTargetPosition(0);
        HLSliderMotor.setTargetPosition(0);
        HRSliderMotor.setPower(0.2);
        HLSliderMotor.setPower(0.2);
        HRSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        HLSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    // ticks loop for arm start
    public void maxH(double turnage) {
        VRSliderMotorTarget = VRSliderMotorTicks * turnage;
        VLSliderMotorTarget = VLSliderMotorTicks * turnage;
        VRSliderMotor.setTargetPosition((int) VRSliderMotorTarget);
        VLSliderMotor.setTargetPosition((int) VLSliderMotorTarget);
        VRSliderMotor.setPower(0.5);
        VLSliderMotor.setPower(0.5);
        VLSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        VRSliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}