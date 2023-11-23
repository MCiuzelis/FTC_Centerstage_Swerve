package org.firstinspires.ftc.teamcode.subsystems;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.hardware.Constants.kd;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ki;
import static org.firstinspires.ftc.teamcode.hardware.Constants.kp;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Constants;

public class SwerveModule extends Thread{

    public static double ticksInOneRad = Constants.ticksInOneRad;
    public static double maxTurningVelocity = Constants.maxTurningVelocity;

    public static double fastCalibrationSpeed = Constants.fastCalibrationSpeed;
    public double slowCalibrationSpeed = Constants.slowCalibrationSpeed;

    DcMotorEx TopMotor, BottomMotor;
    TouchSensor limitSwitch;
    int offset;

    double turnVelocityTicks;
    double driveSpeedMultiplier;
    public ElapsedTime timer = new ElapsedTime();

    public double lastError = 0;
    public double integralSum = 0;

    public SwerveModule(DcMotorEx TopMotor, DcMotorEx BottomMotor, TouchSensor limitSwitch, int offset){
        this.TopMotor = TopMotor;
        this.BottomMotor = BottomMotor;
        this.limitSwitch = limitSwitch;
        this.offset = offset;

    }


    public void updateTurningAndDrivingSpeeds(Rotation2d targetAngle, double moduleSpeed){
        driveSpeedMultiplier = 1;
        double currentAngle = ((TopMotor.getCurrentPosition() + BottomMotor.getCurrentPosition()) / (2f * ticksInOneRad));
        currentAngle = clampAngleUsingTrigReduction(currentAngle, 2d * Math.PI, -2d * Math.PI);

        if (currentAngle * targetAngle.getRadians() < 0){
            if (currentAngle < 0){
                currentAngle += Math.PI; //aka 180 degrees
            }
            else{
                currentAngle -= Math.PI; //aka 180 degrees
            }
            driveSpeedMultiplier *= -1;
        }

        double error = targetAngle.getRadians() - currentAngle;
        if (error > Math.toRadians(95)){
            error = -(Math.PI - error);
            driveSpeedMultiplier *= -1;
        }

        driveSpeedMultiplier *= moduleSpeed;

        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        timer.reset();
        lastError = error;

        turnVelocityTicks = (kp * error) + (ki * integralSum) + (kd * derivative);
        turnVelocityTicks = clamp(turnVelocityTicks, -maxTurningVelocity, maxTurningVelocity);
    }

    public void CalibrateModule(){

        if (limitSwitch.isPressed()) {
            while (getModuleAngleTicks() < 500) {
                setModuleVelocity(fastCalibrationSpeed);
            }
            setModuleVelocity(0);
        }

        while (!limitSwitch.isPressed()) {
            setModuleVelocity(slowCalibrationSpeed);
        }
        setModuleVelocity(0);
        resetModuleEncoders();

        double [] calibrationPoints = new double[3];
        calibrationPoints [0] = getModuleAngleTicks();

        for (int i = 1; i < 3; i++){
            while (getModuleAngleTicks() > -50) {
                setModuleVelocity(-fastCalibrationSpeed);
            }
            setModuleVelocity(0);

            while (!limitSwitch.isPressed()) {
                setModuleVelocity(slowCalibrationSpeed);
            }
            setModuleVelocity(0);
            calibrationPoints[i] = getModuleAngleTicks();
        }

        int finalCalibrationPoint = (int) Math.round((calibrationPoints[0] + calibrationPoints[1] + calibrationPoints[2]) / 3d);
        while (getModuleAngleTicks() > -50) {
            setModuleVelocity(-fastCalibrationSpeed);
        }
        setModuleVelocity(0);

        while (getModuleAngleTicks() < finalCalibrationPoint) {
            setModuleVelocity(slowCalibrationSpeed / 1.75d);
        }
        setModuleVelocity(0);
        resetModuleEncoders();


        if (offset < 0){
            while (getModuleAngleDegrees() > offset) {
                setModuleVelocity(-slowCalibrationSpeed / 1.5d);
            }
        }
        else{
            while (getModuleAngleDegrees() < offset) {
                setModuleVelocity(slowCalibrationSpeed / 1.5d);
            }
        }
        setModuleVelocity(0);
        resetModuleEncoders();
    }

    @Override
    public void run() {
        CalibrateModule();
    }


    private double getModuleAngleTicks(){
        return (TopMotor.getCurrentPosition() + BottomMotor.getCurrentPosition()) / 2d;
    }

    private double getModuleAngleDegrees(){
        return Math.toDegrees(getModuleAngleTicks() / ticksInOneRad);
    }

    private void setModuleVelocity(double velocity){
        TopMotor.setVelocity(velocity);
        BottomMotor.setVelocity(velocity);
    }

    public void resetModuleEncoders(){
        TopMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        TopMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        BottomMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BottomMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        timer.reset();
    }

    public double clampAngleUsingTrigReduction(double angle, double ceiling, double floor){
        while (angle > ceiling) {
            angle -= ceiling;
        }
        while (angle < floor) {
            angle -= floor;
        }
        return angle;
    }
}
