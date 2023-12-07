package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.UpdateClawStateCommand;
import org.firstinspires.ftc.teamcode.commands.InstantCommands.WriteCurrentOffsetAngleToFile;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToDepositPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToPickupPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToTransferPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class AutonomousLeftCommand extends SequentialCommandGroup {
    public AutonomousLeftCommand(ArmSubsystem arm, DrivetrainSubsystem driveTrain, CalibrationTransfer file, RobotHardware robot){
        addCommands(
                new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)
                        .andThen(new WaitCommand(200))
                        .andThen(new SetArmToTransferPositionCommand(arm))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.2, 90, 0, 1060))
                        .andThen(new SetArmToPickupPositionCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED))
                        .andThen(new WaitCommand(400))
                        .andThen(new TurnByDegreesCommand(driveTrain, 86.5, 2, 0.08))
                        .andThen(new WaitCommand(500))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.1, 180, 0, 200))
                        .andThen(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.LEFT_OPENED))

                        .andThen(new WaitCommand(400))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.1, 0, 0, 200))
                        .andThen(new SetArmToTransferPositionCommand(arm))
                        .andThen(new WaitCommand(100))
                        .andThen(new SetArmToDepositPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.LOWPOS))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.18, 6, 0, 1420))

                        .andThen(new WaitCommand(300))
                        .andThen(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN))
                        .andThen(new WaitCommand(200))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.18, 180, 0, 100))
                        .andThen(new SetArmToTransferPositionCommand(arm))
                        .andThen(new WaitCommand(500))
                        .andThen(new SetArmToPickupPositionCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED))
                        .andThen(new WriteCurrentOffsetAngleToFile(file, robot, driveTrain))
        );
    }
}