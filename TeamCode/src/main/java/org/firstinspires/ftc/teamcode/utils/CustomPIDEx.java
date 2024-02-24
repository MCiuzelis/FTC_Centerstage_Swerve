package org.firstinspires.ftc.teamcode.utils;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;

public class CustomPIDEx extends PIDEx {
    public CustomPIDEx(double kP, double kI, double kD, double maxIntegralSum, double stabilityThreshold, double lowPassGain) {
        super(new PIDCoefficientsEx(kP, kI, kD, maxIntegralSum, stabilityThreshold, lowPassGain));
    }

    public void updateCoefficients(double kP, double kI, double kD, double maxIntegralSum, double stabilityThreshold, double lowPassGain){
        super.basedCoefficients = new PIDCoefficientsEx(kP, kI, kD, maxIntegralSum, stabilityThreshold, lowPassGain);
    }
}