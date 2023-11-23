package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class DepositorSubsystem extends SubsystemBase {
    public RobotHardware robot;
    public Telemetry telemetry;
    public boolean DEBUG_MODE;

    public DEPOSITOR_PROBE_STATE depositorProbeState = DEPOSITOR_PROBE_STATE.PICKUP;
    public DEPOSITOR_PIN_STATE depositorPinState = DEPOSITOR_PIN_STATE.RELEASE;
    public SLIDE_ACTUAL_POSITION actual_position = SLIDE_ACTUAL_POSITION.LOWPOS;
    public SLIDE_TARGET_POSITION target_position = SLIDE_TARGET_POSITION.LOWPOS;

    //region SLIDE_PID
    //D and F values are not actually needed in this case
    public boolean isManualControl = false;
    boolean isWithinToleranceLow = false;
    boolean isWithinToleranceHigh = false;
    //0.003d
    public static double P = 0.0d;
    //0.00004d
    public static double I = 0.0d;
    public static double D = 0.0d;
    public static double F = 0.0d;

    private int targetMotorPosition;
    private int currentMotorPosition;
    double lastError = 0;
    double integralSum = 0;
    //endregion
    ElapsedTime timer = new ElapsedTime();



    public DepositorSubsystem(RobotHardware robot, Telemetry telemetry, boolean isMode_DEBUG){
        this.robot = robot;
        this.DEBUG_MODE = isMode_DEBUG;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //initial, setup positions
        update(depositorPinState);
        update(depositorProbeState);
    }

    public void update(DEPOSITOR_PROBE_STATE state) {
        depositorProbeState = state;
        switch (state){
            case DEPOSIT:
                setDepositorServos(Constants.DepositorDepositPosition);
                break;
            case PICKUP:
                setDepositorServos(Constants.DepositorPickupPosition);
                break;
        }
        if (DEBUG_MODE){
            telemetry.addData("Depositor Probe State ", state);
            telemetry.addData("Depositor Probe Servo Left position ", robot.depositorLeftServo.getPosition());
            telemetry.addData("Depositor Probe Servo Right position ", robot.depositorRightServo.getPosition());
            telemetry.update();
        }

    }

    public void update(DEPOSITOR_PIN_STATE state) {
        depositorPinState = state;
        switch (depositorPinState){
            case HOLD:
                robot.depositorPinServo.setPosition(Constants.DepositorPinHoldPosition);
                break;
            case RELEASE:
                robot.depositorPinServo.setPosition(Constants.DepositorPinReleasePosition);
                break;
        }
        if (DEBUG_MODE) {
            telemetry.addData("Depositor Pin State ", state);
            telemetry.addData("Depositor Pin Servo position ", robot.depositorPinServo.getPosition());
            telemetry.update();
        }
    }

    public void update(SLIDE_TARGET_POSITION state) {
        isManualControl = false;
        target_position = state;
        switch (state){
            case LOWPOS:
                targetMotorPosition = Constants.LiftLowPosition;
                break;
            case HIGHPOS:
                targetMotorPosition = Constants.LiftHighPosition;
                break;
        }
    }
    public void update(float power){
        isManualControl = true;
        robot.depositorSlideMotor.setPower(power);
        if (DEBUG_MODE) {
            telemetry.addData("Linear slide motor power", power);
            telemetry.update();
        }
    }

    public void StopSlideMotor(){
        robot.depositorSlideMotor.setPower(0);
        if (DEBUG_MODE){
            telemetry.addLine("Stopping lift motor..!");
            telemetry.update();
        }
    }

    @Override
    public void periodic() {
        if (!isManualControl) {
            AutomatedLiftMovement();
        }
    }
    public void AutomatedLiftMovement(){
        currentMotorPosition = robot.depositorSlideMotor.getCurrentPosition();
        double error = targetMotorPosition - currentMotorPosition;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        if (robot.depositorSlideMotor.getCurrent(CurrentUnit.AMPS) > 10) {
            //robot.depositorSlideMotor.setPower(0);
        }
        if (currentMotorPosition > Constants.LiftHighPosition - Constants.MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.HIGHPOS;
            isWithinToleranceHigh = true;

        }
        else if (currentMotorPosition < Constants.LiftLowPosition + Constants.MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.LOWPOS;
            isWithinToleranceLow = true;
        }
        else {
            isWithinToleranceHigh = false;
            isWithinToleranceLow = false;
        }

        double power = (P * error) + (I * integralSum) + (D * derivative);
        robot.depositorSlideMotor.setPower(power);

        if (DEBUG_MODE) {
            telemetry.addData("CurrentMotorPos", currentMotorPosition);
            telemetry.addData("TargetMotorPos", targetMotorPosition);
            telemetry.addData("ActualMotorPos", actual_position);
            telemetry.addData("Power", power);
            telemetry.addData("Has reached lower tolerance?", isWithinToleranceLow);
            telemetry.addData("Current(Amps)", robot.depositorSlideMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
        timer.reset();
    }

    public void setDepositorServos(double position) {
        robot.depositorRightServo.setPosition(position);
        robot.depositorLeftServo.setPosition(position);
    }


    public boolean WithinToleranceLow(){return isWithinToleranceLow;}
    public boolean WithinToleranceHigh(){return isWithinToleranceHigh;}

    public enum DEPOSITOR_PROBE_STATE{
        PICKUP,
        DEPOSIT
    }
    public enum DEPOSITOR_PIN_STATE{
        RELEASE,
        HOLD
    }
    public enum SLIDE_TARGET_POSITION {
        LOWPOS,
        HIGHPOS
    }
    public enum SLIDE_ACTUAL_POSITION {
        LOWPOS,
        HIGHPOS
    }
}
