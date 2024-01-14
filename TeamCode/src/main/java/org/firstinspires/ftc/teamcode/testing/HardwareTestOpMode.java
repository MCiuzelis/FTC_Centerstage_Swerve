package org.firstinspires.ftc.teamcode.testing;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKCos;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKdSmallError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKiSmallError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKpSmallError;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmKs;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmPidCap;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmStartOffsetAngleRads;
import static org.firstinspires.ftc.teamcode.hardware.Constants.ArmTicksInOneRad;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Disabled
@Config
@TeleOp(name = "HardwareTest", group = "OpMode")
public class HardwareTestOpMode extends OpMode {
    RobotHardware robot = new RobotHardware(hardwareMap);
    //ArmSubsystem arm;
    public static double armTargetPos = 0;
    double previousTime = 0;

    PIDController pid;

    @Override
    public void init() {

        robot.initialiseHardware(telemetry);
        //arm = new ArmSubsystem(robot, telemetry, true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid = new PIDController(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);
    }



    @Override
    public void loop(){
        telemetry.addData("kd", ArmKdSmallError);
        pid.setPID(ArmKpSmallError, ArmKiSmallError, ArmKdSmallError);


//        if (prevTargetAngle != armTargetAngleDegrees) {
//            profileTimer.reset();
//        }




        double currentMotorAngleRads = robot.ArmMotor.getCurrentPosition() / ArmTicksInOneRad + ArmStartOffsetAngleRads;
        double currentMotorAngle = robot.ArmMotor.getCurrentPosition();

        double feedforward = Math.cos(currentMotorAngleRads) * ArmKCos;

        double targetAngleTicks = Math.toRadians(armTargetPos) * ArmTicksInOneRad;

        telemetry.addData("targetMotorAngle", targetAngleTicks);
        double feedback = pid.calculate(currentMotorAngle, targetAngleTicks);

        double friction = Math.signum(targetAngleTicks - currentMotorAngle) * ArmKs;

        double power = feedforward + feedback + friction;

        power = clamp(power, -ArmPidCap, ArmPidCap);

        robot.ArmMotor.setPower(power);

        telemetry.addData("currentMotorPos", robot.ArmMotor.getCurrentPosition());
        telemetry.addData("feedback", feedback);
        telemetry.addData("currentPosition",  currentMotorAngle);
        telemetry.addData("power", power);
        telemetry.addData("targetPosition", armTargetPos);
        telemetry.addData("feedforward", feedforward);
        telemetry.update();
    }






    double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {

        // calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2d;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance)
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt)
            return distance;

        // if we're accelerating
        if (elapsed_time < acceleration_dt)
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration * Math.pow(elapsed_time, 2);

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }





    public double[] gm0profile(double current_velocity, double currentPosition, double targetPosition, double current_time, double maxSpeed, double maxAcceleration) {

        double direction_multiplier = 1;

        double position_error = targetPosition - currentPosition;

        if (position_error < 0) {
            direction_multiplier = -1;
        }

        // if maximum speed has not been reached
        double output_velocity, output_acceleration;

        if (maxSpeed > Math.abs(current_velocity)) {
            output_velocity = current_velocity + direction_multiplier * maxAcceleration * (current_time - previousTime);
            output_acceleration = maxAcceleration;
        }

        //if maximum speed has been reached, stay there for now
        else {
            output_velocity = maxSpeed;
            output_acceleration = 0;
        }

        //#if we are close enough to the object to begin slowing down
        if (position_error <= (output_velocity * output_velocity) / (2 * maxAcceleration)) {
            output_velocity = current_velocity - direction_multiplier * maxAcceleration * (current_time - previousTime);
            output_acceleration = -maxAcceleration;
        }
        previousTime = current_time;
        return new double[]{output_velocity, output_acceleration};
    }
}