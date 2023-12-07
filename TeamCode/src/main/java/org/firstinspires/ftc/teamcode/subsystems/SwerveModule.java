package org.firstinspires.ftc.teamcode.subsystems;
import static com.arcrobotics.ftclib.util.MathUtils.clamp;

import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKa;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKs;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseDriveKv;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKd;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKi;
import static org.firstinspires.ftc.teamcode.hardware.Constants.DriveBaseTurnKp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.highestPossibleMotorVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Constants;

public class SwerveModule extends Thread{

    static double ticksInOneRad = Constants.ticksInOneRad;
    static double maxTurningVelocity = Constants.maxTurningVelocity;

    static double fastCalibrationSpeed = Constants.fastCalibrationSpeed;
    double slowCalibrationSpeed = Constants.slowCalibrationSpeed;

    DcMotorEx TopMotor, BottomMotor;
    TouchSensor limitSwitch;
    int offset;

    double turnVelocityTicks;
    double driveSpeedMultiplier;
    public ElapsedTime timer = new ElapsedTime();

    Telemetry telemetry;

    double lastError = 0;
    double integralSum = 0;

    double angleOffset;




    public SwerveModule(DcMotorEx TopMotor, DcMotorEx BottomMotor, TouchSensor limitSwitch, int offset, double angleOffset, Telemetry telemetry){
        this.TopMotor = TopMotor;
        this.BottomMotor = BottomMotor;
        this.limitSwitch = limitSwitch;
        this.offset = offset;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.angleOffset = angleOffset;
    }




    public void updateTurningAndDrivingSpeeds(Rotation2d targetAngle, double moduleSpeed){
        driveSpeedMultiplier = 1;
        double currentAngle = ((TopMotor.getCurrentPosition() + BottomMotor.getCurrentPosition()) / (2f * ticksInOneRad)) + angleOffset;
        currentAngle = clampAngleAndTranslateRange(currentAngle);

        double error = targetAngle.getRadians() - currentAngle;

        if (Math.abs(error) > Math.toRadians(180)) {
            error += (-Math.signum(error) * Math.toRadians(360));
        }

        if (Math.abs(error) > Math.toRadians(90)){
            error += (-Math.signum(error) * Math.toRadians(180));
            driveSpeedMultiplier *= -1;
        }

        double currentModuleSpeed = (BottomMotor.getVelocity() - TopMotor.getVelocity()) / 2d;

        double driveError = moduleSpeed - currentModuleSpeed;
        double DriveSpeedPower = DriveBaseDriveKs * Math.signum(moduleSpeed) + DriveBaseDriveKv * moduleSpeed + driveError * DriveBaseDriveKa;

        DriveSpeedPower = clamp(DriveSpeedPower, -highestPossibleMotorVelocity, highestPossibleMotorVelocity);
        driveSpeedMultiplier *= DriveSpeedPower;


        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        timer.reset();
        lastError = error;

        turnVelocityTicks = (DriveBaseTurnKp * error) + (DriveBaseTurnKi * integralSum) + (DriveBaseTurnKd * derivative);
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
            while (getAngleDegrees() > offset) {
                setModuleVelocity(-slowCalibrationSpeed / 1.5d);
            }
        }
        else{
            while (getAngleDegrees() < offset) {
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


    public double getAngleRads(){
        return getModuleAngleTicks() / ticksInOneRad;
    }


    private double getAngleDegrees(){
        return Math.toDegrees(getAngleRads());
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
        TopMotor.setPower(0);
        BottomMotor.setPower(0);
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
            angle = -2d * Math.PI + angle;
        }
        if (angle < -Math.PI){
            angle = 2d * Math.PI + angle;
        }
        return angle;
    }
}
