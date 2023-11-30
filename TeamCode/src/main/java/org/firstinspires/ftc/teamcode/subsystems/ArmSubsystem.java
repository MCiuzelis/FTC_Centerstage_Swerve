package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.*;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class ArmSubsystem extends SubsystemBase {

    public RobotHardware robot;
    public Telemetry telemetry;
    public boolean DEBUG_MODE;

    public SLIDE_TARGET_POSITION slide_target_position = SLIDE_TARGET_POSITION.PICKUP_POS;
    public ARM_TARGET_POSITION arm_target_position = ARM_TARGET_POSITION.PICKUP;

    private double armTargetMotorPosition = 0;
    private int slideTargetMotorPosition;
    double slideLastError = 0;
    double slideIntegralSum = 0;
    double armIntegralSum = 0;

    ElapsedTime dedicatedSlideTimer = new ElapsedTime();
    ElapsedTime dedicatedArmTimer = new ElapsedTime();



    public ArmSubsystem(RobotHardware robot, Telemetry telemetry, boolean isMode_DEBUG){
        this.robot = robot;
        this.telemetry = telemetry;
        this.DEBUG_MODE = isMode_DEBUG;
    }


    public void update(CLAW_STATE state) {
        switch (state){
            case LEFT_CLOSED:
                robot.clawLeftServo.setPosition(ClawClosedPosition);
                break;
            case LEFT_OPENED:
                robot.clawLeftServo.setPosition(ClawOpenPosition);
                break;
            case RIGHT_CLOSED:
                robot.clawRightServo.setPosition(ClawClosedPosition);
                break;
            case RIGHT_OPEN:
                robot.clawRightServo.setPosition(ClawOpenPosition);
                break;
            case BOTH_OPEN:
                robot.clawLeftServo.setPosition(ClawOpenPosition);
                robot.clawLeftServo.setPosition(ClawOpenPosition);
                break;
            case BOTH_CLOSED:
                robot.clawLeftServo.setPosition(ClawClosedPosition);
                robot.clawLeftServo.setPosition(ClawClosedPosition);
                break;
        }

    }


    public void update(CLAW_ANGLE angle) {
        switch (angle){
            case PICKUP:
                robot.clawAngleServo.setPosition(ClawAnglePickupPosition);
                break;
            case DEPOSIT:
                robot.clawAngleServo.setPosition(ClawAngleDepositPosition);
                break;
        }
    }


    public void update(SLIDE_TARGET_POSITION state) {
        slide_target_position = state;
        switch (state){
            case PICKUP_POS:
                slideTargetMotorPosition = LiftPickupPosition;
                break;
            case LOWPOS:
                slideTargetMotorPosition = LiftLowPosition;
                break;
            case MIDPOS:
                slideTargetMotorPosition = LiftMidPosition;
                break;
            case HIGHPOS:
                slideTargetMotorPosition = LiftHighPosition;
                break;
        }
    }



    public void update(int offset){
        this.slideTargetMotorPosition += offset;
        slideTargetMotorPosition = clamp(slideTargetMotorPosition, LiftLowPosition, LiftHighPosition);
    }



    public void update(ARM_TARGET_POSITION targetPosition) {
        arm_target_position = targetPosition;
        switch (arm_target_position){
            case DEPOSIT:
                armTargetMotorPosition = armMotorDepositPosition;
                break;
            case PICKUP:
                armTargetMotorPosition = armMotorPickupPosition;
                break;
        }
    }


    public void StopSlideMotor(){
        robot.armMotor.setPower(0);
    }


    @Override
    public void periodic() {
        AutomatedLiftMovement();
        AutomatedArmMovement();
    }



    private void AutomatedArmMovement() {
        int currentMotorPosition = robot.armMotor.getCurrentPosition();
        double error = armTargetMotorPosition - currentMotorPosition;
        //double derivative = (error - slideLastError) / dedicatedArmTimer.seconds();
        armIntegralSum = armIntegralSum + (error * dedicatedArmTimer.seconds());

        double power = (ArmKp * error) + (ArmKi * armIntegralSum);
        robot.armMotor.setPower(power);
        dedicatedArmTimer.reset();
    }



    public void AutomatedLiftMovement(){
        int currentMotorPosition = robot.armMotor.getCurrentPosition();
        double error = slideTargetMotorPosition - currentMotorPosition;
        double derivative = (error - slideLastError) / dedicatedSlideTimer.seconds();
        slideIntegralSum = slideIntegralSum + (error * dedicatedSlideTimer.seconds());


        double power = (LiftKp * error) + (LiftKi * slideIntegralSum) + (LiftKd * derivative) + LiftKf;
        robot.armMotor.setPower(power);
        dedicatedSlideTimer.reset();
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
        DEPOSIT
    }

    public enum SLIDE_TARGET_POSITION {
        PICKUP_POS,
        LOWPOS,
        MIDPOS,
        HIGHPOS,
    }

    public enum ARM_TARGET_POSITION{
        PICKUP,
        DEPOSIT
    }
}