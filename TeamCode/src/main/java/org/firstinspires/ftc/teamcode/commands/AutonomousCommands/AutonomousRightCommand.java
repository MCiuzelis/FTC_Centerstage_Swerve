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

public class AutonomousRightCommand extends SequentialCommandGroup {
    public AutonomousRightCommand(ArmSubsystem arm, DrivetrainSubsystem driveTrain, CalibrationTransfer file, RobotHardware robot){
        addCommands(
                new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)
                        .andThen(new WaitCommand(200))
                        .andThen(new SetArmToTransferPositionCommand(arm))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.2, 50, 0, 1280))
                        .andThen(new WaitCommand(200))
                        .andThen(new TurnByDegreesCommand(driveTrain, 91, 2, 0.08))
                        .andThen(new SetArmToPickupPositionCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED))
                        .andThen(new WaitCommand(200))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.2, 180, 0, 300))
                        .andThen(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.LEFT_OPENED))

                        .andThen(new WaitCommand(500))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.18, 3, 0, 200))
                        .andThen(new WaitCommand(500))
                        .andThen(new SetArmToDepositPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.LOWPOS))
                        .andThen(new WaitCommand(1000))
                        .andThen(new DriveTimeoutCommand(driveTrain, 0.15, -25, 0, 640))

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