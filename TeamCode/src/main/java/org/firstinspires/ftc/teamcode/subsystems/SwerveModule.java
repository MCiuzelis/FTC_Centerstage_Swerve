package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseMaxDriveAcceleration;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKs;
import static org.firstinspires.ftc.teamcode.hardware.Constants.fastCalibrationSpeed;
import static org.firstinspires.ftc.teamcode.hardware.Constants.finalAllMotorVelocityMultiplier;
import static org.firstinspires.ftc.teamcode.hardware.Constants.slowCalibrationSpeed;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ticksInOneRad;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;



public class SwerveModule {
    Telemetry telemetry;

    DcMotorEx TopMotor, BottomMotor;
    DigitalChannel calibrationSensor;
    int calibrationOffset;

    double driveSpeedMultiplier;

    double angleOffset;
    double prevDrivingVelocity;

    boolean DEBUG_MODE;

    Thread startCalibration;



    public SwerveModule(DcMotorEx TopMotor, DcMotorEx BottomMotor, DigitalChannel limitSwitch, int offset, double angleOffset, boolean DEBUG_MODE, Telemetry telemetry) {
        this.TopMotor = TopMotor;
        this.BottomMotor = BottomMotor;
        this.calibrationSensor = limitSwitch;
        this.calibrationOffset = offset;
        this.angleOffset = angleOffset;
        this.DEBUG_MODE = DEBUG_MODE;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }





    public double getTurnCorrection(double targetAngle, PIDEx PID) {
        double currentAngle = normalise(getAngleRads());


        telemetry.addData("currentAngle", currentAngle);
        double adjustedTargetAngle = targetAngle;

        double error = adjustedTargetAngle - currentAngle;

        if (Math.abs(error) > Math.toRadians(180)){
            adjustedTargetAngle -= Math.signum(error) * 2d * Math.PI;
            error = adjustedTargetAngle - currentAngle;
        }

        if (Math.abs(error) > Math.toRadians(95)) {
            adjustedTargetAngle -= Math.signum(error) * Math.PI;
            driveSpeedMultiplier = -1;
        } else driveSpeedMultiplier = 1;


        telemetry.addData("TargetAngle", Math.toDegrees(adjustedTargetAngle));

        if (DEBUG_MODE) {
            telemetry.addData("currentAngle", Math.toDegrees(currentAngle));
            telemetry.addData("TargetAngle", Math.toDegrees(adjustedTargetAngle));
        }

        double frictionCompensation = DriveBaseTurnKs * Math.signum(adjustedTargetAngle - currentAngle);
        return clamp(PID.calculate(adjustedTargetAngle, currentAngle) + frictionCompensation, -1, 1);
    }




    public double getDriveCorrection(double targetVelocity, BasicFeedforward feedforward, PIDEx PID) {
        double error = targetVelocity - prevDrivingVelocity;
        double adjustedTargetVelocity;

        if (Math.abs(error) > DriveBaseMaxDriveAcceleration) {
            double currentVelocityMultiplier = DriveBaseMaxDriveAcceleration / Math.abs(error);
            adjustedTargetVelocity = targetVelocity * currentVelocityMultiplier + prevDrivingVelocity * (1d - currentVelocityMultiplier);
        } else adjustedTargetVelocity = targetVelocity;

        prevDrivingVelocity = adjustedTargetVelocity;
        adjustedTargetVelocity *= driveSpeedMultiplier;

        if (DEBUG_MODE) {
            telemetry.addData("targetVelocity", targetVelocity * driveSpeedMultiplier);
            telemetry.addData("currentVelocity", getDrivingVelocity());
        }

        double output = feedforward.calculate(0, adjustedTargetVelocity, 0)
                + PID.calculate(adjustedTargetVelocity, getDrivingVelocity());
        return clamp(output, -1, 1);
    }




    public void calibrate() {
        startCalibration = new Thread(() -> {
            while (calibrationSensor.getState()) setModuleVelocity(fastCalibrationSpeed);
            while (!calibrationSensor.getState()) setModuleVelocity(fastCalibrationSpeed);

            double startAngle = getAngleDegrees();
            while (getAngleDegrees() < startAngle + 20) setModuleVelocity(fastCalibrationSpeed);
            while (!calibrationSensor.getState()) setModuleVelocity(slowCalibrationSpeed);

            setModuleVelocity(0);
            resetModuleEncoders();
        });
        startCalibration.start();
    }



    public double getAngleTicks() {return (TopMotor.getCurrentPosition() + BottomMotor.getCurrentPosition()) / 2d;}

    public double getAngleRads() {
        telemetry.addData("before offset", getAngleTicks() / ticksInOneRad);
        return getAngleTicks() / ticksInOneRad + angleOffset;
    }

    public double forteleop(){return normalise(getAngleRads());}

    public double getAngleDegrees() {return Math.toDegrees(getAngleRads());}

    public double getTurningVelocity() {return (TopMotor.getVelocity() + BottomMotor.getVelocity()) / 2d;}

    public Rotation2d getAngleRotation2d() {return new Rotation2d(getAngleRads());}

    public double getDrivingVelocity() {return (TopMotor.getVelocity() - BottomMotor.getVelocity()) / 2d;}

    public void setModuleVelocity(double velocity) {
        TopMotor.setVelocity(velocity);
        BottomMotor.setVelocity(velocity);
    }

    public double normalise(double angle) {
        double normalisedAngle = angle;
        telemetry.addData("angle rads", angle);

        while (Math.abs(normalisedAngle) > 2d * Math.PI){
            normalisedAngle -= Math.signum(angle) * 2d * Math.PI;
        }
        if (Math.abs(normalisedAngle) > Math.PI) {
            normalisedAngle -= 2d * Math.PI * Math.signum(normalisedAngle);
        }

        return normalisedAngle;
    }


    public void resetModuleEncoders() {
        stopMotors();
        TopMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TopMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BottomMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BottomMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }


    public void stopMotors() {
        TopMotor.setPower(0);
        BottomMotor.setPower(0);
    }


    public void setPower(double power) {
        TopMotor.setPower(power);
        BottomMotor.setPower(power);
    }


    public void setTurnPower(double turnPower) {
        double output = turnPower * finalAllMotorVelocityMultiplier;
        TopMotor.setPower(output);
        BottomMotor.setPower(output);
    }


    public void setDrivePower(double drivePower) {
        double output = drivePower * finalAllMotorVelocityMultiplier;
        TopMotor.setPower(output);
        BottomMotor.setPower(-output);
    }


    public void setDrivePowerInverted(double drivePower) {
        double output = drivePower * finalAllMotorVelocityMultiplier;
        TopMotor.setPower(-output);
        BottomMotor.setPower(output);
    }


    public void setTurnAndDrivePower(double turnPower, double drivePower) {
        TopMotor.setPower((turnPower + drivePower) * finalAllMotorVelocityMultiplier);
        BottomMotor.setPower((turnPower - drivePower) * finalAllMotorVelocityMultiplier);
    }

    public void setTurnAndDrivePowerInverted(double turnPower, double drivePower) {
        TopMotor.setPower((turnPower - drivePower) * finalAllMotorVelocityMultiplier);
        BottomMotor.setPower((turnPower + drivePower) * finalAllMotorVelocityMultiplier);
    }
}