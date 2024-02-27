package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class SwerveModule implements Runnable{

    static double gobildaMotorGearRatio = (1 + (46d / 17)) * (1 + (46d / 17)) * 28;
    public static double ticksInOneRad = gobildaMotorGearRatio * 62 / (33 * 2 * Math.PI);

    public static double fastCalibrationSpeed = 700;
    public static double slowCalibrationSpeed = 400;

    public static double turnKp = 0.41;
    public static double turnKi = 0.0005;
    public static double turnKd = 0.0003;
    public static double turnPowerCap = 0.45;
    public static double driveKp = 0.00015;
    public static double driveKv = 0.00051;
    public static double maxAcceleration = 420;

    double prevVelocity = 0;

    PIDController turnPID;
    Telemetry telemetry;

    DcMotorEx TopMotor, BottomMotor;
    DigitalChannel calibrationSensor;

    double driveSpeedMultiplier = 1;
    double currentAngleRads = 0;
    boolean DEBUG_MODE;
    public boolean moduleCalibrated = false;


    public SwerveModule(DcMotorEx TopMotor, DcMotorEx BottomMotor, DigitalChannel calibrationSensor, boolean DEBUG_MODE, Telemetry telemetry) {
        this.TopMotor = TopMotor;
        this.BottomMotor = BottomMotor;
        this.calibrationSensor = calibrationSensor;
        this.DEBUG_MODE = DEBUG_MODE;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turnPID = new PIDController(turnKp, turnKi, turnKd);
    }

    public double getTurnCorrection(double targetAngle) {
        updatePID();
        double currentAngle = getNormalisedAngleRads();
        double adjustedTargetAngle = targetAngle;
        double error = adjustedTargetAngle - currentAngle;


        if (Math.abs(error) > Math.PI){
            adjustedTargetAngle -= Math.signum(error) * 2d * Math.PI;
            error = adjustedTargetAngle - currentAngle;
        }


        if (Math.abs(error) > Math.toRadians(95)) {
            adjustedTargetAngle -= Math.signum(error) * Math.PI;
            driveSpeedMultiplier = -1;
        } else driveSpeedMultiplier = 1;

        double power = clamp(turnPID.calculate(currentAngle, adjustedTargetAngle), -turnPowerCap, turnPowerCap);

        if (DEBUG_MODE) {
            telemetry.addData("currentAngle", getAngleDegrees());
            telemetry.addData("TargetAngle", Math.toDegrees(adjustedTargetAngle));
            telemetry.addData("power", power);
        }
        return power;
    }

    public double getDriveCorrection(double targetVelocity) {
        targetVelocity *= driveSpeedMultiplier;

        if (DEBUG_MODE) {
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.addData("currentVelocity", getDrivingVelocity());
        }

        double velocityError = targetVelocity - prevVelocity;
        if (velocityError > maxAcceleration){
            targetVelocity = prevVelocity + maxAcceleration * Math.signum(velocityError);
        }
        prevVelocity = targetVelocity;

        double output = driveKv * targetVelocity + driveKp * (targetVelocity - getDrivingVelocity());
        telemetry.addData("output", output);
        return clamp(output, -1, 1);
    }

    public Vector2d getModuleMotorVelocities(double targetDriveVelocity, double targetAngle, double turnMultiplier){
        return new Vector2d(getDriveCorrection(targetDriveVelocity), getTurnCorrection(targetAngle) * turnMultiplier)
        .rotateBy(-45);
    }

    @Override
    public void run(){
        calibrate();
    }

    public void calibrate() {
        while (calibrationSensor.getState()) setModuleVelocity(fastCalibrationSpeed);
        while (!calibrationSensor.getState()) setModuleVelocity(slowCalibrationSpeed);

        double currentAngle = getAngleDegrees();
        while (getAngleDegrees() < currentAngle + 15) setModuleVelocity(slowCalibrationSpeed);

        if (calibrationSensor.getState()){
            while (calibrationSensor.getState()) setModuleVelocity(fastCalibrationSpeed);
            while (!calibrationSensor.getState()) setModuleVelocity(80);
        }
        else{
            while (!calibrationSensor.getState()) setModuleVelocity(-80);
        }
        setPower(0);
        resetModuleEncoders();
        moduleCalibrated = true;
    }

    public double getAngleTicks() {return (TopMotor.getCurrentPosition() + BottomMotor.getCurrentPosition()) / 2d;}

    public void updateAngle() {currentAngleRads = getAngleTicks() / ticksInOneRad;}

    public double getAngleRads() {return currentAngleRads;}

    public double getNormalisedAngleRads() {return normalise(currentAngleRads);}

    public double getAngleDegrees() {return Math.toDegrees(currentAngleRads);}

    public double getTurningVelocity() {return (TopMotor.getVelocity() + BottomMotor.getVelocity()) / 2d;}

    public Rotation2d getAngleRotation2d() {return new Rotation2d(currentAngleRads);}

    public double getDrivingVelocity() {return (TopMotor.getVelocity() - BottomMotor.getVelocity()) / 2d;}

    public void setModuleVelocity(double velocity) {
        TopMotor.setVelocity(velocity);
        BottomMotor.setVelocity(velocity);
    }

    public double normalise(double angle) {
        double normalisedAngle = angle;

        while (Math.abs(normalisedAngle) > 2d * Math.PI) normalisedAngle -= Math.signum(angle) * 2d * Math.PI;
        if (Math.abs(normalisedAngle) > Math.PI) normalisedAngle -= 2d * Math.PI * Math.signum(normalisedAngle);
        return normalisedAngle;
    }

    public void resetModuleEncoders() {
        stopMotors();
        TopMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TopMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        BottomMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BottomMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopMotors() {
        TopMotor.setPower(0);
        BottomMotor.setPower(0);
    }

    public void setPower(double power) {
        TopMotor.setPower(power);
        BottomMotor.setPower(power);
    }

    public void setPowers(double topPower, double bottomPower) {
        TopMotor.setPower(topPower);
        BottomMotor.setPower(bottomPower);
    }

    public void setTurnPower(double turnPower) {
        TopMotor.setPower(turnPower);
        BottomMotor.setPower(turnPower);
    }

    public void setDrivePower(double drivePower) {
        TopMotor.setPower(drivePower);
        BottomMotor.setPower(-drivePower);
    }

    public void setDrivePowerInverted(double drivePower) {
        TopMotor.setPower(-drivePower);
        BottomMotor.setPower(drivePower);
    }

    public void setTurnAndDrivePower(double turnPower, double drivePower) {
        TopMotor.setPower((turnPower + drivePower));
        BottomMotor.setPower((turnPower - drivePower));
    }

    public void setTurnAndDrivePowerInverted(double turnPower, double drivePower) {
        TopMotor.setPower((turnPower - drivePower));
        BottomMotor.setPower((turnPower + drivePower));
    }

    public void updatePID(){
        turnPID.setPID(turnKp, turnKi, turnKd);
    }
}