package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

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
    boolean isWithinToleranceMid = false;
    boolean isWithinToleranceTransfer = false;
    boolean isWithinToleranceSafeToDeposit = false;
    //0.003d


    private int targetMotorPosition;
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
                setDepositorServos(DepositorDepositPosition);
                break;
            case GOOFY:
                setDepositorServos(DepositorGoofyPosition);
            case PICKUP:
                setDepositorServos(DepositorPickupPosition);
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
            case TRANSFERPOS:
                targetMotorPosition = LiftTransferPosition;
                break;
            case LOWPOS:
                targetMotorPosition = LiftLowPosition;
                break;
            case MIDPOS:
                targetMotorPosition = LiftMidPosition;
                break;
            case HIGHPOS:
                targetMotorPosition = LiftHighPosition;
                break;
            case SAFETODEPOSITPOS:
                targetMotorPosition = LiftSafeToDepositPosition;
                break;
        }
    }


    public void update(int offset){
        this.targetMotorPosition += offset;
        targetMotorPosition = clamp(targetMotorPosition, LiftLowPosition, LiftHighPosition);
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
        int currentMotorPosition = robot.depositorSlideMotor.getCurrentPosition();
        double error = targetMotorPosition - currentMotorPosition;
        double derivative = (error - lastError) / timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        if (currentMotorPosition > LiftHighPosition - MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.HIGHPOS;
            isWithinToleranceHigh = true;
        }
        else if (currentMotorPosition < LiftLowPosition + MaxAllowedLiftError && currentMotorPosition > LiftLowPosition - MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.LOWPOS;
            isWithinToleranceLow = true;
        }
        else if (currentMotorPosition > LiftMidPosition -MaxAllowedLiftError && currentMotorPosition < LiftMidPosition + MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.MIDPOS;
            isWithinToleranceMid = true;
        }
        else if(currentMotorPosition < LiftTransferPosition + MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.TRANSFERPOS;
            isWithinToleranceSafeToDeposit = true;
            isWithinToleranceTransfer=true;
        }
        else if (currentMotorPosition < LiftSafeToDepositPosition + MaxAllowedLiftError){
            actual_position = SLIDE_ACTUAL_POSITION.SAFETODEPOSITPOS;
            isWithinToleranceSafeToDeposit = true;
        }
        else {
            isWithinToleranceHigh = false;
            isWithinToleranceLow = false;
            isWithinToleranceMid = false;
            isWithinToleranceTransfer = false;
            isWithinToleranceSafeToDeposit = false;
        }

        double power = (LiftKp * error) + (LiftKi * integralSum) + (LiftKd * derivative) + LiftKf;
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
    public boolean WithinToleranceTransfer(){return isWithinToleranceTransfer;}
    public boolean WithinToleranceMid(){return isWithinToleranceMid;}
    public boolean WithinToleranceSafeToDeposit(){return isWithinToleranceSafeToDeposit;}

    public enum DEPOSITOR_PROBE_STATE{
        PICKUP,
        GOOFY,
        DEPOSIT
    }
    public enum DEPOSITOR_PIN_STATE{
        RELEASE,
        HOLD
    }
    public enum SLIDE_TARGET_POSITION {
        TRANSFERPOS,
        LOWPOS,
        MIDPOS,
        HIGHPOS,
        SAFETODEPOSITPOS
    }
    public enum SLIDE_ACTUAL_POSITION {
        LOWPOS,
        HIGHPOS,
        MIDPOS,
        TRANSFERPOS,
        SAFETODEPOSITPOS
    }
}