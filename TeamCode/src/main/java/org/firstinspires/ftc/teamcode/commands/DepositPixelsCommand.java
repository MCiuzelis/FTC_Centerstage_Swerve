package org.firstinspires.ftc.teamcode.commands;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DepositPixelsCommand extends SequentialCommandGroup {
    public DepositPixelsCommand(ArmSubsystem arm, DrivetrainSubsystem robot){

        addCommands(
                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN),
                new WaitCommand(150),
                new InstantCommand(()-> arm.offsetLift(-50)),
                new WaitCommand(50),
                new DriveForSecondsCommand(robot, new Vector2d(-0.3, 0), 0.4),
                new SetArmToStateCommand(arm, SetArmToStateCommand.ArmState.TRANSFER),
                new WaitUntilCommand(arm::areAxonsCloseToTransferPos),
                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)
        );
        addRequirements(arm, robot);
    }
}