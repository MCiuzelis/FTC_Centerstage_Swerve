package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmToPositionCommand extends InstantCommand {
    public SetArmToPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.ARM_TARGET_POSITION ARM_POSITION){
        super(()-> armSubsystem.update(ARM_POSITION));
    }
}
