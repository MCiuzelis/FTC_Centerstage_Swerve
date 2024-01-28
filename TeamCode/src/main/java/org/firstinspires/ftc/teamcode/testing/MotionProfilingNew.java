package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.hardware.Constants.clawPickupPos;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.utils.CustomMotionProfile;

@Disabled
//@Config
@TeleOp(name = "MotionProfileTestNEW", group = "OpMode")
public class MotionProfilingNew extends OpMode {
    DcMotorEx testMotor;
    Servo servo;
    PIDEx controller;

    double targetPos = 0;

    public static double targetLow = 40;
    public static double targetHigh = 350;

    public static double kP = 0.0025;
    public static double kI = 0.0009;
    public static double kD = 0.0001;
    public static double kV = 0.00052;
    public static double kA = 0.000102;
    public static double kS = 0.01;
    public static double kCos = 0.6;

    public static double maxIntegralSum = 0.1;
    public static double stabilityThreshold = 0.1;
    public static double lowPassGain = 0.8;

    public static double maxVelocity = 710;
    public static double maxAcceleration = 14000;
    public static double maxJerk = 23000;

    CustomMotionProfile motionProfile;




    @Override
    public void init() {
        testMotor = hardwareMap.get(DcMotorEx.class, "ch0");
        testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        servo = hardwareMap.get(Servo.class, "CHservo0");
        servo.setPosition(clawPickupPos);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motionProfile = new CustomMotionProfile(telemetry, maxVelocity, maxAcceleration, maxJerk, kV, kA, kS, kCos, controller);
        controller = new PIDEx(new PIDCoefficientsEx(kP, kI, kD, maxIntegralSum, stabilityThreshold, lowPassGain));
    }





    @Override
    public void loop(){
        motionProfile.updateCoefficients(maxVelocity, maxAcceleration, maxJerk, kV, kA,kS, kCos, controller);

        if (gamepad1.cross){
            targetPos = targetLow;
        }
        else if (gamepad1.triangle){
            targetPos = targetHigh;
        }
        else if (gamepad1.square){
            testMotor.setPower(0);
            testMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        double currentPos = getPosition();
        double currentVel = getVelocity();

        motionProfile.getJerkLimitedMotionProfileOutput(currentPos, currentVel, targetPos, testMotor);
        telemetry.update();
    }






    public double getPosition(){
        return testMotor.getCurrentPosition();
    }


    public double getVelocity(){
        return testMotor.getVelocity();
    }
}
