package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Disabled
@TeleOp(name = "⚠️🚨Only if robot REALLY BROKEN🚨⚠️", group = "OpMode")
public class DebugRobot extends OpMode {
    RobotHardware hardware;
    ArmSubsystem arm;
    DrivetrainSubsystem swerve;
    CalibrationTransfer file;
    boolean pushedCalibrationOffsets = false;



    @Override
    public void init() {
        hardware = new RobotHardware(hardwareMap);
        hardware.initialiseHardware(telemetry);

        swerve = new DrivetrainSubsystem(hardware, telemetry, false, false);
        arm = new ArmSubsystem(hardware, telemetry, false);
        file = new CalibrationTransfer(telemetry);
    }





    @Override
    public void loop(){
        swerve.FrontLeft.setTurnPower(gamepad1.left_stick_x * 0.5d);
        swerve.FrontRight.setTurnPower(gamepad1.right_stick_x * 0.5d);

        if (gamepad1.left_bumper){
            swerve.Back.setTurnPower(0.3d);
        }
        else if (gamepad1.right_bumper){
            swerve.Back.setTurnPower(-0.3d);
        }

        else if (gamepad1.left_trigger > 0){
            hardware.SlideMotor.setPower(gamepad1.left_trigger * 0.5d);
        }
        else if (gamepad1.right_trigger > 0){
            hardware.SlideMotor.setPower(gamepad1.right_trigger * -0.5d);
        }
        else{
            swerve.stopAllMotors();
            swerve.FrontLeft.stopMotors();
            swerve.FrontRight.stopMotors();
            swerve.Back.stopMotors();
            hardware.SlideMotor.setPower(0);
        }



        if (gamepad1.triangle && !pushedCalibrationOffsets){
            file.PushCalibrationData(hardware.imu.getRotation2d().getRadians(), swerve.getAllModuleAngleRads());
            pushedCalibrationOffsets = true;
        }
        else if (!gamepad1.triangle){
            pushedCalibrationOffsets = false;
        }
    }
}
