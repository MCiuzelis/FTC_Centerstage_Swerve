
package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Constants.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.Timer;

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
        update(HINGE_SERVO_TARGET_STATE.CHILLIN);
    }

    public void update(BROOM_MOTOR_STATE state){
        broom_motor_state = state;
        switch (state){
            case ACTIVE:
                robot.intakeBroomMotor.setPower(-BroomMotorPower_normal);
                break;
            case DISABLED:
                robot.intakeBroomMotor.setPower(0);
                break;
            case REVERSED:
                robot.intakeBroomMotor.setPower(BroomMotorPower_reversed);
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
                robot.intakeHingeServo.setPosition(HingeServoPickupPos);
                break;
            case TRANSFER:
                robot.intakeHingeServo.setPosition(HingeServoTransferPos);
                break;
            case CHILLIN:
                robot.intakeHingeServo.setPosition(HingeServoChillPos);
                break;
        }

    }

    @Override
    public void periodic() {
        if (robot.limitIntake.isPressed()){
            hinge_servo_actual_state = HINGE_SERVO_ACTUAL_STATE.TRANSFER;
        }
        else if (!robot.limitIntake.isPressed()){hinge_servo_actual_state = HINGE_SERVO_ACTUAL_STATE.PICKUP;}
        else {hinge_servo_actual_state = HINGE_SERVO_ACTUAL_STATE.CHILLIN;}

        if (DEBUG_MODE){
            telemetry.addData("Hinge Servo State", hinge_servo_target_state);
            telemetry.addData("Hinge Servo actual state", hinge_servo_actual_state);
            telemetry.addData("Is button pressed?", robot.limitIntake.isPressed());
            telemetry.addData("Hinge Servo Position", robot.intakeHingeServo.getPosition());
            telemetry.addData("Is ready to lock?", ReadyToLock());
        }

    }

    public boolean ReadyToLock() {
        if (hinge_servo_actual_state == HINGE_SERVO_ACTUAL_STATE.TRANSFER || hinge_servo_actual_state == HINGE_SERVO_ACTUAL_STATE.CHILLIN){ return true;}
        else{ return false;}
    }
    public boolean IntakeServoStuck(){
        ElapsedTime timer = new ElapsedTime();

        if (hinge_servo_target_state == HINGE_SERVO_TARGET_STATE.TRANSFER && hinge_servo_actual_state != HINGE_SERVO_ACTUAL_STATE.TRANSFER && !robot.limitIntake.isPressed() && timer.seconds() >= 3){
            timer.reset();
            return true;
        }
        return false;
    }

    public enum BROOM_MOTOR_STATE{
        ACTIVE,
        DISABLED,
        REVERSED
    }
    public enum HINGE_SERVO_TARGET_STATE {
        PICKUP,
        TRANSFER,
        CHILLIN
    }
    public enum HINGE_SERVO_ACTUAL_STATE {
        PICKUP,
        TRANSFER,
        CHILLIN,
        STUCK
    }


}
