package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.LowPassFilter;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class ArmSubsystem extends SubsystemBase {

    //SLIDES:
    public static double maxVelocity = 2350;
    public static double minVelocity = 350;
    public static double slowDownDistance = 150;

    public static double kP = 0.0115;
    public static double kD = 0;
    public static double kG = 0.005;
    public static double lowPassGain = 0.6;

    public static double slidePickupPos = 0;
    public static double slideLowPos = 120;
    public static double slideMidPos = 750;
    public static double slideHighPos = 1380;
    public static double slideErrorMargin = 7;

    //MAIN ARM:
    public static double axonHighPos = 0.75;
    public static double axonMidPos = 0.83;
    public static double axonLowPos = 0.88;
    public static double axonPickupPos = 0;
    public static double axonTransferPos = 0.03;
    public static double axonUpperTransferPos = 0.07;

    //CLAW ANGLE:
    public static double clawPickupPos = 0.42;
    public static double clawLowPos = 0.78;
    public static double clawTransferPos = 0.04;
    public static double clawMidPos = 0.71;
    public static double clawHighPos = 0.67;

    //CLAW:
    public static double clawClosedPosition = 0;
    public static double clawOpenPosition = 1;

    public static double clawProximitySensorVoltageThreshold = 1;

    double prevSlideTargetPos = 0;
    double prevMotorPos = 0;
    double slideTargetPos = 0;
    double startingPosition = 0;

    boolean isClawLeftLocked = false;
    boolean isClawRightLocked = false;

    RobotHardware robot;
    Telemetry telemetry;
    boolean DEBUG_MODE;

    PIDController slidePid = new PIDController(kP, 0, kD);
    LowPassFilter slideFilter = new LowPassFilter(lowPassGain);
    ElapsedTime slideProfileTimer = new ElapsedTime();




    public ArmSubsystem(RobotHardware hardware, Telemetry telemetry, boolean DEBUG_MODE){
        this.robot = hardware;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.DEBUG_MODE = DEBUG_MODE;
        update(CLAW_ANGLE.PICKUP);
        update(AXON_STATE.PICKUP);
        update(SLIDE_STATE.PICKUP);
        update(CLAW_STATE.BOTH_CLOSED);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    public void invertClawState(){
        if (!isClawLeftLocked) {
            if (robot.clawLeftServo.getPosition() == clawClosedPosition) update(CLAW_STATE.LEFT_OPENED);
            else update(CLAW_STATE.LEFT_CLOSED);
        }

        if (!isClawRightLocked) {
            if (robot.clawRightServo.getPosition() == clawClosedPosition) update(CLAW_STATE.RIGHT_OPEN);
            else update(CLAW_STATE.RIGHT_CLOSED);
        }
    }

    public enum CLAW{
        LEFT,
        RIGHT
    }

    public void changeClawLockState (CLAW claw, boolean locked){
        if (claw == CLAW.LEFT) isClawLeftLocked = locked;
        else if (claw == CLAW.RIGHT) isClawRightLocked = locked;
    }

    public void update(CLAW_STATE state) {
        switch (state){
            case LEFT_CLOSED:
                robot.clawLeftServo.setPosition(clawClosedPosition);
                break;
            case LEFT_OPENED:
                robot.clawLeftServo.setPosition(clawOpenPosition);
                break;
            case RIGHT_CLOSED:
                robot.clawRightServo.setPosition(clawClosedPosition);
                break;
            case RIGHT_OPEN:
                robot.clawRightServo.setPosition(clawOpenPosition);
                break;
            case BOTH_OPEN:
                robot.clawLeftServo.setPosition(clawOpenPosition);
                robot.clawRightServo.setPosition(clawOpenPosition);
                break;
            case BOTH_CLOSED:
                robot.clawLeftServo.setPosition(clawClosedPosition);
                robot.clawRightServo.setPosition(clawClosedPosition);
                break;
        }
    }

    public void update(CLAW_ANGLE angle) {
        switch (angle){
            case PICKUP:
                robot.clawAngleServo.setPosition(clawPickupPos);
                break;
            case LOWPOS:
                robot.clawAngleServo.setPosition(clawLowPos);
                break;
            case TRANSFER:
                robot.clawAngleServo.setPosition(clawTransferPos);
                break;
            case MIDPOS:
                robot.clawAngleServo.setPosition(clawMidPos);
                break;
            case HIGHPOS:
                robot.clawAngleServo.setPosition(clawHighPos);
                break;
        }
    }

    public void update(SLIDE_STATE state) {
        switch (state){
            case PICKUP:
                slideTargetPos = slidePickupPos;
                break;
            case LOW:
                slideTargetPos = slideLowPos;
                break;
            case MID:
                slideTargetPos = slideMidPos;
                break;
            case HIGH:
                slideTargetPos = slideHighPos;
                break;
        }
    }

    public void update(AXON_STATE targetPosition){
        switch (targetPosition){
            case LOWPOS:
                robot.axonLeft.setPosition(axonLowPos);
                robot.axonRight.setPosition(axonLowPos);
                break;

            case TRANSFER:
                robot.axonLeft.setPosition(axonTransferPos);
                robot.axonRight.setPosition(axonTransferPos);
                break;

            case UPPER_TRANSFER:
                robot.axonLeft.setPosition(axonUpperTransferPos);
                robot.axonRight.setPosition(axonUpperTransferPos);
                break;

            case MIDPOS:
                robot.axonLeft.setPosition(axonMidPos);
                robot.axonRight.setPosition(axonMidPos);
                break;

            case HIGHPOS:
                robot.axonLeft.setPosition(axonMidPos);
                robot.axonRight.setPosition(axonHighPos);
                break;

            case PICKUP:
                robot.axonLeft.setPosition(axonPickupPos);
                robot.axonRight.setPosition(axonPickupPos);
                break;
        }
    }

    @Override
    public void periodic() {
        slidePid.setPID(kP, 0, kD);

        double currentMotorPos = robot.SlideMotor.getCurrentPosition();

        if (slideTargetPos != prevSlideTargetPos){
            startingPosition = currentMotorPos;
            slideProfileTimer.reset();
        }

        prevSlideTargetPos = slideTargetPos;
        double error = slideTargetPos - currentMotorPos;

        if (currentMotorPos <= slowDownDistance && !(prevMotorPos <= slowDownDistance)){
            startingPosition = currentMotorPos;
            slideProfileTimer.reset();
        }
        prevMotorPos = currentMotorPos;


        double velocity = maxVelocity;
        if (error < 0) {
            velocity = currentMotorPos <= slowDownDistance ? minVelocity : maxVelocity;
        }

        velocity = slideFilter.estimate(velocity);
        double newPosition = startingPosition + Math.signum(error) * slideProfileTimer.seconds() * velocity;

        if ((error >= 0 && newPosition >= slideTargetPos) || (error <= 0 && newPosition <= slideTargetPos)){
            newPosition = slideTargetPos;
        }

        double power = slidePid.calculate(currentMotorPos, newPosition) + kG;
        robot.SlideMotor.setPower(power);

        telemetry.addData("isLeftLocked", isClawLeftLocked);
        telemetry.addData("isRightLocked", isClawRightLocked);


        if (DEBUG_MODE) {
            double armAngleRads = robot.armAxonEncoder.getVoltage() / 1.65d * Math.PI;
            telemetry.addData("AxonAngle", Math.toDegrees(armAngleRads));

            telemetry.addData("GamepadTargetMotorAngle", slideTargetPos);
            telemetry.addData("actualTargetPos", newPosition);
            telemetry.addData("currentAngle", currentMotorPos);
            telemetry.addData("error", Math.signum(error));
            telemetry.addData("power", power);
        }
    }

    public void offsetLift(double offset){
        slideTargetPos += offset;
    }

    public boolean areSlidesDown(){
        return (robot.SlideMotor.getCurrentPosition() > slidePickupPos - slideErrorMargin &&
                robot.SlideMotor.getCurrentPosition() < slidePickupPos + slideErrorMargin);
    }

    private double getAxonAngleDegrees(){
        return robot.armAxonEncoder.getVoltage() / 3.3 * 360;
    }

    public boolean areAxonsCloseToTransferPos(){
        return getAxonAngleDegrees() > 200;
    }

    public boolean areAxonsAtPickup(){
        return getAxonAngleDegrees() > 275;
    }

    public boolean leftPixelInClaw() {return robot.clawProximityLeft.getVoltage() < clawProximitySensorVoltageThreshold;}

    public boolean rightPixelInClaw() {return robot.clawProximityRight.getVoltage() < clawProximitySensorVoltageThreshold;}

    public enum SLIDE_STATE{
        PICKUP,
        LOW,
        MID,
        HIGH
    }

    public enum CLAW_STATE{
        LEFT_OPENED,
        LEFT_CLOSED,
        RIGHT_OPEN,
        RIGHT_CLOSED,
        BOTH_OPEN,
        BOTH_CLOSED
    }

    public enum CLAW_ANGLE{
        PICKUP,
        TRANSFER,
        LOWPOS,
        MIDPOS,
        HIGHPOS
    }

    public enum AXON_STATE {
        PICKUP,
        LOWPOS,
        MIDPOS,
        TRANSFER,
        UPPER_TRANSFER,
        HIGHPOS
    }
}