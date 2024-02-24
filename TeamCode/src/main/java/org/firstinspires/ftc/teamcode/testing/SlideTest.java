package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.MotionProfiling.PositionMotionProfile;

@Disabled
@Config
@Photon
@TeleOp(name = "Slide Test", group = "OpMode")
public class SlideTest extends OpMode {

    DcMotorEx SlideMotor;

    public static double maxVelocity = 1500;
    public static double minVelocity = 250;
    public static double slowDownDistance = 200;

    public static double kP = 0.015;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.0125;

    public static double lowPassGain = 0.9;
    double slideTargetPos = 0;


    //double prevSlideTargetPos = 0;
    //double prevMotorPos = 0;
    //double startingPosition = 0;

    //PIDController pid = new PIDController(kP, kI, kD);
    //LowPassFilter filter = new LowPassFilter(lowPassGain);
    //ElapsedTime profileTimer = new ElapsedTime();

    //
    PositionMotionProfile positionMotionProfile = new PositionMotionProfile(minVelocity, maxVelocity, slowDownDistance, lowPassGain, kP, kI, kD);

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SlideMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
    }



    @Override
    public void loop() {
        if (gamepad1.left_bumper){
            slideTargetPos = 20;
        }
        else if (gamepad1.right_bumper){
            slideTargetPos = 600;
        }

        SlideMotor.setPower(positionMotionProfile.getPower(SlideMotor.getCurrentPosition(), slideTargetPos) + kG);
//        double currentMotorPos = SlideMotor.getCurrentPosition();
//
//        if (slideTargetPos != prevSlideTargetPos){
//            startingPosition = currentMotorPos;
//            profileTimer.reset();
//        }
//
//        prevSlideTargetPos = slideTargetPos;
//
//        double error = slideTargetPos - currentMotorPos;
//
//        if (currentMotorPos <= slowDownDistance && !(prevMotorPos <= slowDownDistance)){
//            startingPosition = currentMotorPos;
//            profileTimer.reset();
//        }
//        prevMotorPos = currentMotorPos;
//
//
//        double velocity = maxVelocity;
//        if (error < 0) {
//            velocity = currentMotorPos <= slowDownDistance ? minVelocity : maxVelocity;
//        }
//
//        velocity = filter.estimate(velocity);
//        double newPosition = startingPosition + Math.signum(error) * profileTimer.seconds() * velocity;
//
//        if ((error > 0 && newPosition > slideTargetPos) || (error < 0 && newPosition < slideTargetPos)){
//            newPosition = slideTargetPos;
//        }
//
//        double power = pid.calculate(currentMotorPos, newPosition) + kG;
//        SlideMotor.setPower(power);
//
//
//
//
//        telemetry.addData("GamepadtargetMotorAngle", slideTargetPos);
//        telemetry.addData("actualTargetPos", newPosition);
//        telemetry.addData("currentAngle", currentMotorPos);
//        telemetry.addData("error", Math.signum(error));
//        telemetry.addData("power", power);
    }
}