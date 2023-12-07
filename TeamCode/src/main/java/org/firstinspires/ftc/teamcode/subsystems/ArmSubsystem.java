package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
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

    ElapsedTime dedicatedSlideTimer = new ElapsedTime();
    ElapsedTime dedicatedArmTimer = new ElapsedTime();

    PIDController armPidController;



    public ArmSubsystem(RobotHardware robot, Telemetry telemetry, boolean isMode_DEBUG){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.DEBUG_MODE = isMode_DEBUG;
        armPidController = new PIDController(ArmKp, ArmKi, ArmKd);
        update(ARM_TARGET_POSITION.PICKUP);
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
                robot.clawRightServo.setPosition(ClawOpenPosition);
                break;
            case BOTH_CLOSED:
                robot.clawLeftServo.setPosition(ClawClosedPosition);
                robot.clawRightServo.setPosition(ClawClosedPosition);
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
            case TRANSFER:
                robot.clawAngleServo.setPosition(ClawAngleTransferPosition);
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





    public void update(ARM_TARGET_POSITION targetPosition) {
        arm_target_position = targetPosition;
        switch (arm_target_position){
            case DEPOSIT:
                armTargetMotorPosition = armMotorDepositPosition;
                break;
            case TRANSFER:
                armTargetMotorPosition = armMotorTransferPosition;
                break;
            case PICKUP:
                armTargetMotorPosition = armMotorPickupPosition;
                break;
        }
    }




    @Override
    public void periodic() {
        armPidController.setPID(ArmKp, ArmKi, ArmKd);
        AutomatedLiftMovement();
        AutomatedArmMovement();

        telemetry.addData("time", Runtime.getRuntime());
    }




    private void AutomatedArmMovement() {
        int currentMotorPosition = robot.armMotor.getCurrentPosition();
        double currentArmAngle = (currentMotorPosition / armMotorTicksInOneRad) - ArmStartAngle;
        double power = armPidController.calculate(armTargetMotorPosition, currentMotorPosition) + Math.cos(currentArmAngle) * ArmKf;

        power = clamp(power, -ArmPowerClamp, ArmPowerClamp);
        robot.armMotor.setPower(power);
        dedicatedArmTimer.reset();
    }




    public void AutomatedLiftMovement(){
        int currentMotorPosition = robot.slideMotor.getCurrentPosition();
        double error = slideTargetMotorPosition - currentMotorPosition;
        double derivative = (error - slideLastError) / dedicatedSlideTimer.seconds();
        slideIntegralSum = slideIntegralSum + (error * dedicatedSlideTimer.seconds());


        double power = (LiftKp * error) + (LiftKi * slideIntegralSum) + (LiftKd * derivative) + LiftKf;
        power = clamp(power, -LiftPowerClamp, LiftPowerClamp);
        robot.slideMotor.setPower(power);
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
        TRANSFER,
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
        TRANSFER,
        DEPOSIT
    }
}