package org.firstinspires.ftc.teamcode.utils.MotionProfiling;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class CustomMotionProfile {
    double maxVel;
    double maxAccel;
    double maxJerk;

    double kV, kS, kA, kCos;
    Telemetry telemetry;

    double isMotionProfileInvertedMultiplier = 1;
    double currentZeroPosition = 0;
    MotionProfile profile;
    ElapsedTime motionProfileTimer;

    double prevTime = 0;
    double prevTargetPosition = 0;
    PIDEx pid;

    double ArmKCos = 0;
    double ArmStartOffsetAngleRads = 0;
    double ArmTicksInOneRad = 0;




    public CustomMotionProfile(Telemetry telemetry, double maxVelocity, double maxAcceleration, double kV, PIDEx pid){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        maxVel = maxVelocity;
        maxAccel = maxAcceleration;
        this.kV = kV;
        this.pid = pid;
    }




    public CustomMotionProfile(Telemetry telemetry, double maxVelocity, double maxAcceleration, double maxJerk, double kV, double kA, double kS, double kCos, PIDEx pid){
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        maxVel = maxVelocity;
        maxAccel = maxAcceleration;
        this.maxJerk = maxJerk;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kCos = kCos;
        this.pid = pid;
        motionProfileTimer = new ElapsedTime();
    }




    public void updateCoefficients(double maxVelocity, double maxAcceleration, double maxJerk, double kV, double kA, double kS, double kCos, PIDEx pid){
        maxVel = maxVelocity;
        maxAccel = maxAcceleration;
        this.maxJerk = maxJerk;
        this.kV = kV;
        this.kA = kA;
        this.kS = kS;
        this.kCos = kCos;
        this.pid = pid;
    }






    public void getJerkLimitedMotionProfileOutput(double currentPosition, double currentVelocity, double targetPosition, DcMotorEx motor){

        if (targetPosition != prevTargetPosition) {
            double positionError = targetPosition - currentPosition;
            isMotionProfileInvertedMultiplier = Math.signum(positionError);
            positionError = Math.abs(positionError);
            currentZeroPosition = currentPosition;

            profile = MotionProfileGenerator.generateSimpleMotionProfile(
                    new MotionState(0, currentVelocity, 0),
                    new MotionState(positionError, 0, 0),
                    maxVel,
                    maxAccel,
                    maxJerk
            );
            motionProfileTimer.reset();
        }

        double velocityFeedforward, accelerationFeedforward, positionFeedback;

        MotionState state;
        if (profile != null) {

            state = profile.get(motionProfileTimer.seconds());
            velocityFeedforward = state.getV() * kV * isMotionProfileInvertedMultiplier;
            accelerationFeedforward = state.getA() * kA * isMotionProfileInvertedMultiplier;
            positionFeedback = pid.calculate(state.getX(), Math.abs(currentPosition - currentZeroPosition)) * isMotionProfileInvertedMultiplier;

            double frictionCompensation = Math.signum(targetPosition - currentPosition) * kS;
            double currentMotorAngleRads = currentPosition / ArmTicksInOneRad + ArmStartOffsetAngleRads;
            double feedforward = Math.cos(currentMotorAngleRads) * ArmKCos;
            double power = positionFeedback + velocityFeedforward + accelerationFeedforward + frictionCompensation + feedforward;


            if (state.getV() == 0 && state.getA() == 0){
                motor.setTargetPosition((int) targetPosition);
                motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                motor.setPower(0.15);
            }
            else{
                motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
                motor.setPower(power);
            }


            telemetry.addData("state_X", state.getX() * isMotionProfileInvertedMultiplier);
            telemetry.addData("state_V", state.getV() * isMotionProfileInvertedMultiplier);
            telemetry.addData("state_A", state.getA() * isMotionProfileInvertedMultiplier);
            telemetry.addData("correction", power);
        }


        else{
            //positionFeedback = pid.calculate(targetPosition, currentPosition);
            telemetry.addData("state_X", 0);
            telemetry.addData("state_V", 0);
            telemetry.addData("state_A", 0);
            telemetry.addData("correction", 0);
        }




        telemetry.addData("targetPos", targetPosition);
        telemetry.addData("currentPos", currentPosition);
        telemetry.addData("currentVel", currentVelocity);
        telemetry.addData("time", motionProfileTimer.seconds());

        prevTargetPosition = targetPosition;
    }







    public double getAccelerationLimitedVelocityProfileOutput(double targetVelocity, double currentVelocity, double currentTime){
        double dt = currentTime - prevTime;
        double targetAccel = (targetVelocity - currentVelocity) / dt;

        double target;
        if (Math.abs(targetAccel) > maxAccel) target = currentVelocity + Math.signum(targetVelocity) * dt * maxAccel;
        else target = targetVelocity;

        return clamp(target, -maxVel, maxVel) * kV + pid.calculate(target, currentVelocity);
    }
}