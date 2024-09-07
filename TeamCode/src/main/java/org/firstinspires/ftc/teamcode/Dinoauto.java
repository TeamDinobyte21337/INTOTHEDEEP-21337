
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Autonomous (name = "Dinobyters", group = "Linear OpMpde")

public class Dinoauto extends LinearOpMode{

    //tuning for velocity variables begin
    private double integralSum = 0;
    private double Kp = 0;
    private double Ki = 0;
    private double Kd = 0;
    private double Kf = 0;
    //tuning Kf first is essential
    //tuning for velocity variables ends

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // mapping hardware + initialize end

        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setDirection(DcMotorEx.Direction.REVERSE);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // mapping hardware + initialize end

        waitForStart();

        if (opModeIsActive()){

        int desiredPosition = 100;
        backRightMotor.setTargetPosition(desiredPosition);
        backRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        backLeftMotor.setTargetPosition(desiredPosition);
        backLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontLeftMotor.setTargetPosition(desiredPosition);
        frontLeftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        frontRightMotor.setTargetPosition(desiredPosition);
        frontRightMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        //drives forward X inches
        }

        }

    public static class Odometry {
        // Constants
        public final double ENCODER_WHEEL_DIAMETER = 1.37795;

        // Variables
        private double xPos, yPos;
        public double angle;
        private double lastLeftEnc = 0, lastNormalEnc = 0;

        public Odometry(double xPos, double yPos) {
            this.xPos = xPos;
            this.yPos = yPos;
        }

        public void updatePosition(double l, double n, double ang) {
            double dL = l - lastLeftEnc;
            double dN = n - lastNormalEnc;
            lastNormalEnc = n;
            lastLeftEnc = l;

            // ticks measured after
            double ENCODER_TICKS_PER_REVOLUTION = 8154;
            double ENCODER_WHEEL_CIRCUMFERENCE = Math.PI * 2.0 * (ENCODER_WHEEL_DIAMETER * 0.5);
            double leftDist = -dL * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;
            double dyR = leftDist;
            double dxR = -dN * ENCODER_WHEEL_CIRCUMFERENCE / ENCODER_TICKS_PER_REVOLUTION;

            //double cos = Math.cos((angle.degrees_to_radians(ang)));
            //double sin = Math.sin((angle.degrees_to_radians(ang)));
            //double dx = (dxR * sin) + (dyR * cos);
            //double dy = (-dxR * cos) + (dyR * sin);

            angle = ang;
            //xPos += dx;
           // yPos += dy;
        }

        public double getX() {
            return xPos;
        }

        public double getY() {
            return yPos;
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




