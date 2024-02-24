package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.utils.MotionProfiling.AccelerationMotionProfile;

@Disabled
@Config
@Photon
@TeleOp(name = "Acceleration profile test", group = "OpMode")
public class MotionProfileTEst extends OpMode {

    DcMotorEx SlideMotor;

    public static double maxAcceleration = 50;

    public static double kV = 0.000325;
    public static double kP = 0.00035;
    public static double kI = 0;
    public static double kD = 0;

    public static double targetVelocity = 0;


    AccelerationMotionProfile accelerationMotionProfile;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        SlideMotor = hardwareMap.get(DcMotorEx.class, "testMotor");
        accelerationMotionProfile = new AccelerationMotionProfile(maxAcceleration, kV, kP, kI, kD, telemetry);
    }



    @Override
    public void loop() {
        accelerationMotionProfile.updateConstants(maxAcceleration, kV, kP, kI, kD);
        if (gamepad1.left_bumper){
            targetVelocity = 200;
        }
        else if (gamepad1.right_bumper){
            targetVelocity = 2000;
        }
        double currentVelocity = SlideMotor.getVelocity();
        double power = accelerationMotionProfile.getPower(currentVelocity, targetVelocity);

        SlideMotor.setPower(power);
        telemetry.addData("target velocity", targetVelocity);
        telemetry.addData("current velocity", currentVelocity);
        telemetry.addData("power", power);
    }
}