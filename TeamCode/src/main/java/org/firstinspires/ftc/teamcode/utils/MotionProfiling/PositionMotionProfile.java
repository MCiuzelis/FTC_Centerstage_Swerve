package org.firstinspires.ftc.teamcode.utils.MotionProfiling;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PositionMotionProfile {

    double prevPosition = 0;
    double startingPos = 0;
    ElapsedTime timer = new ElapsedTime();

    double slowDownDistance;
    double minVelocity, maxVelocity;

    LowPassFilter lowPassFilter;
    PIDController pidController;

    public PositionMotionProfile(double minVelocity, double maxVelocity, double slowDownDistance,
                                 double lowPassGain, double kP, double kI, double kD){

        this.slowDownDistance = slowDownDistance;
        this.minVelocity = minVelocity;
        this.maxVelocity = maxVelocity;
        pidController = new PIDController(kP, kI, kD);
        lowPassFilter = new LowPassFilter(lowPassGain);
    }


    public PositionMotionProfile(double minVelocity, double maxVelocity,
                                 double lowPassGain, double kP, double kI, double kD) {

        this (minVelocity, maxVelocity, 0, lowPassGain, kP, kI, kD);
    }


    public double getPower(double currentPos, double targetPos) {
        if (targetPos != prevPosition) {
            startingPos = currentPos;
            timer.reset();
        }

        double error = targetPos - currentPos;
        double prevError = targetPos - prevPosition;

        double velocity = maxVelocity;

        if (Math.abs(error) <= slowDownDistance && !(Math.abs(prevError) <= slowDownDistance)) {
            velocity = minVelocity;
            startingPos = currentPos;
            timer.reset();
        }
        prevPosition = currentPos;

        velocity = lowPassFilter.estimate(velocity);
        double newPosition = startingPos + Math.signum(error) * timer.seconds() * velocity;


        if (targetPos > startingPos && newPosition > targetPos) {
            newPosition = targetPos;
        } else if (targetPos < startingPos && newPosition < targetPos) {
            newPosition = targetPos;
        }
        return pidController.calculate(currentPos, newPosition);
    }


    public void updatePID (double kP, double kI, double kD){
        pidController.setPID(kP, kI, kD);
    }
}