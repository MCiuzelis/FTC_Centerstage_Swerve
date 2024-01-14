package org.firstinspires.ftc.teamcode.testing;

import static org.firstinspires.ftc.teamcode.hardware.Constants.planeLaunchPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.SetArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Disabled
@Config
@TeleOp(name = "MainOpModeWriting", group = "OpMode")
public class MainTeleOpModeWriting extends CommandOpMode {
    RobotHardware hardware = new RobotHardware(hardwareMap);
    DrivetrainSubsystem swerve;
    CalibrationTransfer file = new CalibrationTransfer(telemetry);
    ArmSubsystem armSubsystem;
    GamePad gamePad;
    ElapsedTime timer;

    Thread armThread;


    @Override
    public void initialize() {
        hardware.initialiseHardware(telemetry);

        armSubsystem = new ArmSubsystem(hardware,telemetry,false);
        swerve = new DrivetrainSubsystem(hardware, telemetry, false);

        //swerve = new DrivetrainSubsystem(hardware, telemetry, new double[]{Math.toRadians(90), Math.toRadians(90), Math.toRadians(90)}, 0, false);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamePad = new GamePad(gamepad1);
        swerve.resetAllEncoders();

        gamePad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.HIGHPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.MIDPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.PICKUP));
        gamePad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.LOWPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.TRANSFER));
        gamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_OPEN), new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(new SetClawAngleCommand(armSubsystem, ArmSubsystem.CLAW_ANGLE.PICKUP), new SetClawAngleCommand(armSubsystem, ArmSubsystem.CLAW_ANGLE.HIGHPOS));

        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }



    @Override
    public void run(){
        hardware.clearBulkCache();
        super.run();


        if (timer == null){
            hardware.startIMUThread(this);
            //if (armThread == null){
                    //armThread = new Thread(armSubsystem);
                    //armThread.start();
            //}
            timer = new ElapsedTime();
        }
        swerve.drive(gamePad.getJoystickVector(), gamePad.getTurnSpeed());



        if (gamepad1.options){
            hardware.planeServo.setPosition(planeLaunchPosition);
        }


        file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
        //telemetry.addData("robot Angle", hardware.imuAngle.getDegrees());
        //telemetry.update();
        //armSubsystem.periodic();
    }
}