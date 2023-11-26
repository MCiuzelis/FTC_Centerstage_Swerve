package org.firstinspires.ftc.teamcode.subsystems;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKd;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKi;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    Telemetry telemetry;

    public double lastError = 0;
    public double integralSum = 0;

    public SwerveModule(DcMotorEx TopMotor, DcMotorEx BottomMotor, TouchSensor limitSwitch, int offset, Telemetry telemetry){
        this.TopMotor = TopMotor;
        this.BottomMotor = BottomMotor;
        this.limitSwitch = limitSwitch;
        this.offset = offset;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public void updateTurningAndDrivingSpeeds(Rotation2d targetAngle, double moduleSpeed){
        driveSpeedMultiplier = 1;
        double currentAngle = ((TopMotor.getCurrentPosition() + BottomMotor.getCurrentPosition()) / (2f * ticksInOneRad));
        currentAngle = clampAngleAndTranslateRange(currentAngle);

        telemetry.addData("currentAnglebeforescuff", Math.toDegrees(currentAngle));


//        if (currentAngle < Math.toRadians(-90) && targetAngle.getRadians() > Math.toRadians(90)){
//            currentAngle += Math.toRadians(360);
//        }
//
//        else if (currentAngle > Math.toRadians(90) && targetAngle.getRadians() < Math.toRadians(-90)){
//            currentAngle -= Math.toRadians(360);
//        }

        double error = targetAngle.getRadians() - currentAngle;

        if (Math.abs(error) > Math.toRadians(200)){     //teoriskai tuetu 180 but
            currentAngle = currentAngle - Math.signum(currentAngle) * Math.toRadians(360);
            error = targetAngle.getRadians() - currentAngle;
        }


//        if (error > Math.toRadians(105)){
//            error = -(Math.PI - error);
//            driveSpeedMultiplier *= -1;
//        }

        driveSpeedMultiplier *= moduleSpeed;

        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        timer.reset();
        lastError = error;

        turnVelocityTicks = (DriveBaseTurnKp * error) + (DriveBaseTurnKi * integralSum) + (DriveBaseTurnKd * derivative);
        turnVelocityTicks = clamp(turnVelocityTicks, -maxTurningVelocity, maxTurningVelocity);

        telemetry.addData("current", Math.toDegrees(currentAngle));
        telemetry.addData("target", targetAngle.getDegrees());
        telemetry.addData("error", error);
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

    public double clampAngleAndTranslateRange(double angle){
        while (angle >= 2d * Math.PI) {
            angle -= 2d * Math.PI;
        }
        while (angle <= -2d * Math.PI) {
            angle += 2d * Math.PI;
        }

        if (angle > Math.PI){
            angle = 2d * Math.PI - angle;
        }
        if (angle < -Math.PI){
            angle = 2d * Math.PI + angle;
        }
        return angle;
    }
}
