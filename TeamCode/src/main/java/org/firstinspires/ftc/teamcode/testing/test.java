package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.hardware.Constants.ticksInOneRad;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PropDetectionProcessor;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.testing.roadRunner.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

@Disabled
@Config
@Autonomous(name = "test auto")

public class test extends CommandOpMode {

    DrivetrainSubsystem swerve;
    RobotHardware hardware;
    GamePad gamePad;
    ArmSubsystem arm;
    CalibrationTransfer file;


    ElapsedTime timer = new ElapsedTime();


    @Override
    public void initialize() {
        hardware = new RobotHardware(hardwareMap);
        gamePad = new GamePad(gamepad1);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hardware.initialiseHardware(telemetry);
        file = new CalibrationTransfer(telemetry);

        swerve = new DrivetrainSubsystem(hardware, telemetry, false);

        arm = new ArmSubsystem(hardware, telemetry, true);
        swerve.resetAllEncoders();

        hardware.startIMUThread(this);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }
    public void run(){
        hardware.clearBulkCache();


        swerve.drive(gamePad.getJoystickVector().scale(hardware.getDriveMultiplierFromBatteryVoltage()), gamePad.getTurnSpeed());
        swerve.periodic();

        //telemetry.addData("test", Math.toDegrees(swerve.getAllModuleAngleRads()[0]));
        //telemetry.update();

        if (timer.seconds() > 4) {
            file.PushCalibrationData(0, swerve.getAllModuleAngleRads());
            telemetry.addLine("pushed");
            telemetry.update();
            timer.reset();
        }
    }

}