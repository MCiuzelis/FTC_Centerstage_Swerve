package org.firstinspires.ftc.teamcode.testing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
//@Config
@TeleOp(name = "MotionProfileTest", group = "OpMode")
public class MotionProfilingTest extends OpMode {
    DcMotorEx testMotor;
    PIDEx controller;
    LowPassFilter filter;
    ElapsedTime timer = new ElapsedTime();
    ElapsedTime motionProfileTimer = new ElapsedTime();

    MotionProfile profile;
    MotionState state;

    double invertMotionProfileMultiplier = 1;

    double prevTime = 0;
    double prevVelocity = 0;

    double targetPos = 0;
    double prevTargetPos = 0;

    double currentZeroPosition = 0;

    public static double targetLow = 0;
    public static double targetHigh = 20000;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;
    public static double kV = 0.00036;
    public static double kA = 0;
    public static double kS = 0;

    public static double maxIntegralSum = 0;
    public static double stabilityThreshold = 0;
    public static double lowPassGain = 0;

    public static double accelerationLowPassGain = 0.85;

    public static double maxVelocity = 2300;
    public static double maxAcceleration = 2000;
    public static double maxJerk = 3000;




    @Override
    public void init() {
        filter = new LowPassFilter(accelerationLowPassGain);
        testMotor = hardwareMap.get(DcMotorEx.class, "ch0");
        testMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    @Override
    public void start(){
        timer.reset();
        motionProfileTimer.reset();
    }



    @Override
    public void loop(){
        controller = new PIDEx(new PIDCoefficientsEx(kP, kI, kD, maxIntegralSum, stabilityThreshold, lowPassGain));

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
        //double currentAccel = getAcceleration();

        if (targetPos != prevTargetPos) {
             double positionError = targetPos - currentPos;
             invertMotionProfileMultiplier = Math.signum(positionError);
             positionError = Math.abs(positionError);
             currentZeroPosition = currentPos;

             profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(0, currentVel, 0),
                    new MotionState(positionError, 0, 0),
                    maxVelocity,
                    maxAcceleration,
                    maxJerk
            );
             motionProfileTimer.reset();
        }

        double velocityFeedforward = 0;
        double accelerationFeedforward = 0;
        double positionFeedback;

        if (profile != null) {
            state = profile.get(motionProfileTimer.seconds());
            velocityFeedforward = state.getV() * kV * invertMotionProfileMultiplier;
            accelerationFeedforward = state.getA() * kA * invertMotionProfileMultiplier;
            positionFeedback = controller.calculate(state.getX(), Math.abs(currentPos - currentZeroPosition)) * invertMotionProfileMultiplier;
        }
        else{
            positionFeedback = controller.calculate(targetPos, currentPos);
        }

        if (state == null){
            telemetry.addData("state_X", 0);
            telemetry.addData("state_V", 0);
            telemetry.addData("state_A", 0);
        }
        else {
            telemetry.addData("state_X", state.getX() * invertMotionProfileMultiplier);
            telemetry.addData("state_V", state.getV() * invertMotionProfileMultiplier);
            telemetry.addData("state_A", state.getA() * invertMotionProfileMultiplier);
        }

        double frictionCompensation = Math.signum(targetPos - currentPos) * kS;

        double correction = positionFeedback + velocityFeedforward + accelerationFeedforward + frictionCompensation;


        telemetry.addData("targetPos", targetPos);
        telemetry.addData("currentPos", currentPos);
        telemetry.addData("currentVel", currentVel);
        //telemetry.addData("currentAccel", currentAccel);
        telemetry.addData("correction", correction);
        telemetry.addData("time", motionProfileTimer.seconds());

        testMotor.setPower(correction);
        prevTargetPos = targetPos;
    }






    public double getPosition(){
        return testMotor.getCurrentPosition();
    }


    public double getVelocity(){
        return testMotor.getVelocity();
    }



    public double getAcceleration(){
        double dt = timer.seconds() - prevTime;
        double currentVelocity = testMotor.getVelocity();
        double output = (currentVelocity - prevVelocity) / dt;
        output = filter.estimate(output);
        prevVelocity = currentVelocity;
        prevTime = timer.seconds();
        return output;
    }
}
