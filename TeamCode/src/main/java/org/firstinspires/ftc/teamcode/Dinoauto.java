package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@Autonomous (name = "Dinoauto" +
        "", group = "Linear OpMpde")

public class Dinoauto extends LinearOpMode {

    // declare motors start
    private DcMotor FLMotor;
    private DcMotor FRMotor;
    private DcMotor BLMotor;
    private DcMotor BRMotor;
    // declare motor end

    //tuning for velocity variables begin
    private double integralSum = 0;
    private double Kp = 17;
    private double Ki = 0.15;
    private double Kd = 1;
    private double Kf = 0;
    //tuning Kf first is essential
    //tuning for velocity variables ends

    // encoder for chamber slider start
    double slider1Ticks = 1425;
    double slider1Target;
    // encoder defining for chamber slider end

    // encoder for net slider start
    double slider2Ticks = 1425;
    double slider2Target;
    // encoder defining for net slider end

    // encoder for arm start
    double armTicks = 860.32;
    double armTarget;
    // encoder for arm end

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // mapping wheel begins
        FLMotor = (DcMotorEx) hardwareMap.dcMotor.get("FLMotor");
        FRMotor = (DcMotorEx) hardwareMap.dcMotor.get("FRMotor");
        BLMotor = (DcMotorEx) hardwareMap.dcMotor.get("BLMotor");
        BRMotor = (DcMotorEx) hardwareMap.dcMotor.get("BRMotor");
        // mapping wheel ends

        waitForStart();

        // PID control beings
        while (opModeIsActive()) {
            double power = PIDControl(100, FLMotor.getCurrentPosition());
            FLMotor.setPower(power);
            FRMotor.setPower(power);
            BLMotor.setPower(power);
            BRMotor.setPower(power);
        }
        // PID control end

        // moves robot forwards two tiles
        encoderMovement(539, 0, 0, .4);
        sleep(500);

        // strafes robot one tile
        encoderMovement(0, 290, 0, .4);

       // ADD SLIDER EXTEND AND CLAWS

        // moves robot backwards two tiles
        encoderMovement(-539, 0, 0, .4);

        // strafes robot backwards one tile
        encoderMovement(0, -290, 0, .4);
        sleep(500);
    }

    // loop for autonomous begins
    public void encoderMovement(int forward, int strafe, int turn, double power) {

        FLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // We have entered three parameters into our position, but really we are going to set all but one of them to zero
        FLMotor.setTargetPosition(-forward - strafe + turn);
        FRMotor.setTargetPosition(forward + strafe - turn);
        BLMotor.setTargetPosition(-forward - strafe - turn);
        BRMotor.setTargetPosition(+forward - strafe - turn);

        // setting the power to whatever we input into the function
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);

        FLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BLMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BRMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (BLMotor.isBusy() || BRMotor.isBusy() || FLMotor.isBusy() || FRMotor.isBusy()) {
            telemetry.addLine(String.valueOf(BLMotor.getCurrentPosition()));
            telemetry.addLine(String.valueOf(BRMotor.getCurrentPosition()));
            telemetry.addLine(String.valueOf(FLMotor.getCurrentPosition()));
            telemetry.addLine(String.valueOf(FRMotor.getCurrentPosition()));
            telemetry.update();
        }

        sleep(150);
    }
    // loop for autonomous ends

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + ( reference * Kf);
    }

}