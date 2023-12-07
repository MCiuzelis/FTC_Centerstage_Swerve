package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.*;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToDepositPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToPickupPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToTransferPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Config
@TeleOp(name = "MainTeleOpModeWModuleAngleReading", group = "OpMode")
public class MainTeleOpModeWithModuleAngleReading extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    DrivetrainSubsystem drivetrainSubsystem;
    GamePad gamePad;


    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        CalibrationTransfer file = new CalibrationTransfer(telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, file.pullModuleAngleOffsets(), file.getRobotHeadingOffset(), true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamePad = new GamePad(gamepad1);
        ArmSubsystem arm = new ArmSubsystem(robot, telemetry, false);

        PlaneLauncherSubsystem plane = new PlaneLauncherSubsystem(robot, telemetry);
        plane.update(PlaneLauncherSubsystem.PLANE_STATE.LOCK);

        robot.changeBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        drivetrainSubsystem.resetAllEncoders();





        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED));
        gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetArmToPickupPositionCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN));
        gamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SetArmToTransferPositionCommand(arm));
        gamePad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetArmToDepositPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.MIDPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new SetArmToDepositPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.HIGHPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new InstantCommand(()-> plane.update(PlaneLauncherSubsystem.PLANE_STATE.LAUNCH)));

    }

    @Override
    public void run(){
        robot.clearBulkCache();
        drivetrainSubsystem.drive(gamePad.getJoystickVector(), gamePad.getTurnSpeed());
        super.run();

        telemetry.update();
    }
}