package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp (name = "Dinobyters", group = "Linear OpMpde")

    public class Dinobyters extends LinearOpMode{

    //tuning for velocity variables

    private double integralSum = 0;
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double Kf = 0;

    // tuning Kf first is essential

    //tuning for velocity variables ends

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // mapping hardware + initialize end

        DcMotorEx FLMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        FLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx FRMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        FRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx BRMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        BRMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx BLMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        BLMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FLMotor.setDirection(DcMotorEx.Direction.FORWARD);
        FRMotor.setDirection(DcMotorEx.Direction.REVERSE);
        BLMotor.setDirection(DcMotorEx.Direction.FORWARD);
        BRMotor.setDirection(DcMotorEx.Direction.REVERSE);

        // mapping hardware + initialize end

        waitForStart();

        //PID control power

        while (opModeIsActive()) {
        double power = PIDControl(100, FLMotor.getCurrentPosition());
        FLMotor.setPower(power);
        FRMotor.setPower(power);
        BLMotor.setPower(power);
        BRMotor.setPower(power);

        //setting power to motors when driving

        double forward = -.80 * gamepad1.left_stick_y;
        double strafe = -.60 * gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double slowDown = -.3 * gamepad1.left_stick_y;

        // when the driver presses the "A" button, the robot's motors will slow down
        // less power :3

        if (gamepad1.a){
            FLMotor.setPower(slowDown);
            FRMotor.setPower(slowDown);
            BLMotor.setPower(slowDown);
            BRMotor.setPower(slowDown);
        }

        FLMotor.setPower(-forward + strafe - turn);
        FRMotor.setPower(forward + strafe - turn);
        BLMotor.setPower(-forward - strafe - turn);
        BRMotor.setPower(+forward - strafe - turn);

        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        return (error * Kp) + (derivative * Kd) + (integralSum * Ki) + ( reference * Kf);
          }
       }





