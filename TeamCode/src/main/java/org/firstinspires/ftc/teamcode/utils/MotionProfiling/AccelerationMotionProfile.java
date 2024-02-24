package org.firstinspires.ftc.teamcode.utils.MotionProfiling;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AccelerationMotionProfile {

    double prevVelocity = 0;
    double startingVelocity = 0;
    ElapsedTime timer = new ElapsedTime();

    double maxAcceleration;
    PIDController pidController;

    double kV;
    Telemetry telemetry;


    public AccelerationMotionProfile(double maxAcceleration, double kV, double kP, double kI, double kD, Telemetry telemetry){
        this.maxAcceleration = maxAcceleration;
        this.kV = kV;
        pidController = new PIDController(kP, kI, kD);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }



    public double getPower(double currentVel, double targetVel) {
        if (targetVel != prevVelocity) {
            startingVelocity = currentVel;
            timer.reset();
        }
        prevVelocity = targetVel;

        double newVelocity = startingVelocity + Math.signum(targetVel - startingVelocity) * timer.seconds() * maxAcceleration;

        if (targetVel > startingVelocity && newVelocity > targetVel) {
            newVelocity = targetVel;
        } else if (targetVel < startingVelocity && newVelocity < targetVel) {
            newVelocity = targetVel;
        }

        telemetry.addData("calculated velocity", newVelocity);
        telemetry.addData("current velocity", currentVel);

        return newVelocity;
    }



    public void updateConstants(double maxAcceleration, double kV, double kP, double kI, double kD){
        this.maxAcceleration = maxAcceleration;
        this.kV = kV;
        pidController.setPID(kP, kI, kD);
    }
}