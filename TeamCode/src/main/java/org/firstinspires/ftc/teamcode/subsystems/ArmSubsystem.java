package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmHighPosAngle;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKCos;
//import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKdLargeError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKdSmallError;
//import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKiLargeError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKiSmallError;
//import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKpLargeError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKpSmallError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKs;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmLowPosAngle;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmMidPosAngle;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmPickupAngle;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmPidCap;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmStartOffsetAngleRads;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmTicksInOneRad;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmTransferPosAngle;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ClawClosedPosition;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ClawHighPosPosition;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ClawLowPosPosition;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ClawMidPosPosition;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ClawOpenPosition;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ClawPickupPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
public class ArmSubsystem extends SubsystemBase implements Runnable{

    RobotHardware robot;
    Telemetry telemetry;
    boolean DEBUG_MODE;
    ARM_TARGET_POSITION arm_target_position;
    double armTargetAngle;
    PIDController pid;
    public static double currentMotorAngle;


    public ArmSubsystem(RobotHardware hardware, Telemetry telemetry, boolean DEBUG_MODE){
        this.robot = hardware;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid = new PIDController(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
        this.DEBUG_MODE = DEBUG_MODE;
        update(CLAW_STATE.BOTH_CLOSED);
        update(CLAW_ANGLE.PICKUP);
        //update(ARM_TARGET_POSITION.PICKUP);
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
                robot.clawAngleServo.setPosition(ClawPickupPosition);
                break;
            case LOWPOS:
                robot.clawAngleServo.setPosition(ClawLowPosPosition);
                break;
            case MIDPOS:
                robot.clawAngleServo.setPosition(ClawMidPosPosition);
                break;
            case HIGHPOS:
                robot.clawAngleServo.setPosition(ClawHighPosPosition);
                break;
        }
    }


    @Override
    public void run(){
        while (true) periodic();
    }

    public void update(ARM_TARGET_POSITION targetPosition) {

        switch (targetPosition){
            case PICKUP:
//                if (Math.abs(currentMotorAngle-ArmPickupAngle) > 20){
//                    pid.setPID(ArmKpLargeError, ArmKiLargeError, ArmKdLargeError);
//                }
//                else {
//                    pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
//                }
                update(CLAW_ANGLE.PICKUP);
                armTargetAngle = ArmPickupAngle;
                break;
            case LOWPOS:
//                if (Math.abs(currentMotorAngle-ArmLowPosAngle) > 20){
//                    pid.setPID(ArmKpLargeError, ArmKiLargeError, ArmKdLargeError);
//                }
//                else {
//                    pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
//                }
                update(CLAW_ANGLE.LOWPOS);
                armTargetAngle = ArmLowPosAngle;
                break;
            case MIDPOS:
//                if (Math.abs(currentMotorAngle-ArmMidPosAngle) > 20){
//                    pid.setPID(ArmKpLargeError, ArmKiLargeError, ArmKdLargeError);
//                }
//                else {
//                    pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
//                }
                update(CLAW_ANGLE.MIDPOS);
                armTargetAngle = ArmMidPosAngle;
                break;
            case HIGHPOS:
//                if (Math.abs(currentMotorAngle-ArmHighPosAngle) > 20){
//                    pid.setPID(ArmKpLargeError, ArmKiLargeError, ArmKdLargeError);
//                }
//                else {
//                    pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
//                }
                update(CLAW_ANGLE.HIGHPOS);
                armTargetAngle = ArmHighPosAngle;
                break;
            case TRANSFER:
//                if (Math.abs(currentMotorAngle-ArmTransferPosAngle) > 20){
//                    pid.setPID(ArmKpLargeError, ArmKiLargeError, ArmKdLargeError);
//                }
//                else {
//                    pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
//                }
                update(CLAW_ANGLE.PICKUP);
                armTargetAngle = ArmTransferPosAngle;
                break;
        }
    }






    @Override
    public void periodic() {
//        double currentMotorAngle = robot.ArmMotor.getCurrentPosition() / ArmTicksInOneRad + ArmStartOffsetAngleRads;
//
//        PIDCoefficients coefficients = new PIDCoefficients(ArmKp, ArmKi, ArmKd);
//        PIDFController controller = new PIDFController(coefficients, ArmKv, ArmKa, 0);
//
//        if (prevArmTargetAngle != armTargetAngle) {
//             profile = MotionProfileGenerator.generateSimpleMotionProfile(
//                    new MotionState(currentMotorAngle, 0, 0),
//                    new MotionState(Math.toRadians(armTargetAngle), 0, 0),
//                    5,
//                    2,
//                    0
//             );
//             profileTimer.reset();
//        }
//
//        prevArmTargetAngle = armTargetAngle;
//        MotionState state = profile.get(profileTimer.seconds());
//
//        controller.setTargetPosition(state.getX());
//        controller.setTargetVelocity(state.getV());
//        controller.setTargetAcceleration(state.getA());
//
//        double power = controller.update(currentMotorAngle) + ArmKCos * Math.cos(currentMotorAngle);


//
//        //armPid = new BasicPID(new PIDCoefficients(ArmKp, ArmKi, ArmKd));
//
//        telemetry.addData("current", Math.toDegrees(currentMotorAngle));
//        telemetry.addData("target", armTargetAngle);
//
//        //double power = armPid.calculate(Math.toRadians(armTargetAngle), currentMotorAngle) + ArmKCos * Math.cos(currentMotorAngle);
//
//        telemetry.addData("power", power);
//
//        robot.ArmMotor.setPower(power);

        //pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
        double currentMotorAngleRads = robot.ArmMotor.getCurrentPosition() / ArmTicksInOneRad + ArmStartOffsetAngleRads;
        currentMotorAngle = robot.ArmMotor.getCurrentPosition();

        double feedforward = Math.cos(currentMotorAngleRads) * ArmKCos;

        double targetAngleTicks = Math.toRadians(armTargetAngle) * ArmTicksInOneRad;

        //telemetry.addData("targetMotorAngle", targetAngleTicks);
        //telemetry.addData("currentAngle", currentMotorAngle);

        double feedback = pid.calculate(currentMotorAngle, targetAngleTicks);
        double friction = Math.signum(targetAngleTicks - currentMotorAngle) * ArmKs;
        double power = feedforward + feedback + friction;
        power = clamp(power, -ArmPidCap, ArmPidCap);

        robot.ArmMotor.setPower(power);
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
        LOWPOS,
        MIDPOS,
        HIGHPOS
    }

    public enum ARM_TARGET_POSITION{
        PICKUP,
        LOWPOS,
        MIDPOS,
        TRANSFER,
        HIGHPOS
    }
}