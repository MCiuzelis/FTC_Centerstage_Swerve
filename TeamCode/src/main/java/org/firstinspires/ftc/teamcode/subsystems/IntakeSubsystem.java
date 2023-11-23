
package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Constants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class IntakeSubsystem extends SubsystemBase{
    public RobotHardware robot;
    public Telemetry telemetry;
    public boolean DEBUG_MODE;

    BROOM_MOTOR_STATE broom_motor_state = BROOM_MOTOR_STATE.DISABLED;
    HINGE_SERVO_TARGET_STATE hinge_servo_target_state = HINGE_SERVO_TARGET_STATE.PICKUP;
    HINGE_SERVO_ACTUAL_STATE hinge_servo_actual_state;



    public IntakeSubsystem(RobotHardware hw, Telemetry telemetry, boolean isMode_DEBUG) {
        robot = hw;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DEBUG_MODE = isMode_DEBUG;
    }

    public void update(BROOM_MOTOR_STATE state){
        broom_motor_state = state;
        switch (state){
            case ACTIVE:
                robot.intakeBroomMotor.setPower(-Constants.BroomMotorPower_normal);
                break;
            case DISABLED:
                robot.intakeBroomMotor.setPower(0);
                break;
            case REVERSED:
                robot.intakeBroomMotor.setPower(Constants.BroomMotorPower_reversed);
                break;
        }
        if (DEBUG_MODE){
            telemetry.addData("Broom motor power", robot.intakeBroomMotor.getPower());
        }
    }

    public void update(HINGE_SERVO_TARGET_STATE state) {
        hinge_servo_target_state = state;
        switch (state){
            case PICKUP:
                robot.intakeHingeServo.setPosition(Constants.HingeServoPickupPos);
                break;
            case TRANSFER:
                robot.intakeHingeServo.setPosition(Constants.HingeServoTransferPos);
                break;
        }

    }

    @Override
    public void periodic() {
        if (robot.limitIntake.isPressed()){
            hinge_servo_actual_state = HINGE_SERVO_ACTUAL_STATE.TRANSFER;
        }
        else {hinge_servo_actual_state = HINGE_SERVO_ACTUAL_STATE.PICKUP;}

        if (DEBUG_MODE){
            telemetry.addData("Hinge Servo State", hinge_servo_target_state);
            telemetry.addData("Hinge Servo actual state", hinge_servo_actual_state);
            telemetry.addData("Is button pressed?", robot.limitIntake.isPressed());
            telemetry.addData("Hinge Servo Position", robot.intakeHingeServo.getPosition());
            telemetry.addData("Is ready to lock?", ReadyToLock());
        }

    }

    public boolean ReadyToLock() {
        if (hinge_servo_actual_state == HINGE_SERVO_ACTUAL_STATE.TRANSFER){ return true;}
        else{ return false;}
    }

    public enum BROOM_MOTOR_STATE{
        ACTIVE,
        DISABLED,
        REVERSED
    }
    public enum HINGE_SERVO_TARGET_STATE {
        PICKUP,
        TRANSFER
    }
    public enum HINGE_SERVO_ACTUAL_STATE {
        PICKUP,
        TRANSFER
    }


}
