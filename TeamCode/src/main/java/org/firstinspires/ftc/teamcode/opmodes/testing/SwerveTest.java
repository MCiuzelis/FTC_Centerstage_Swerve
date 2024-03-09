package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem.highestPossibleMotorVelocity;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Photon
@Disabled
@TeleOp(name = "swerve 5.0 test", group = "OpMode")
public class SwerveTest extends OpMode {

    RobotHardware robotHardware;
    DrivetrainSubsystem swerve;
    double angle = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.initialiseHardware(telemetry);

        swerve = new DrivetrainSubsystem(robotHardware, telemetry, false, false);
        swerve.FrontRight.calibrate();
    }



    @Override
    public void loop(){
        robotHardware.getCalibrationSensorTelemetry();
        double currentAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double magnitude = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);

        if (magnitude != 0){
            angle = currentAngle;
        }

        double power = swerve.Back.getTurnCorrection(angle);
        //swerve.Back.setTurnPower(power);

        //double power = Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x);
        //swerve.FrontLeft.setDrivePower(swerve.FrontLeft.getDriveCorrection(power * highestPossibleMotorVelocity, 0));
    }
}
