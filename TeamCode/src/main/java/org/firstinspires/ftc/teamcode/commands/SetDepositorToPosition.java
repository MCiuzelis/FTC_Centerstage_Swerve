package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetDepositorToPosition extends SequentialCommandGroup {
    public SetDepositorToPosition(ArmSubsystem armSubsystem, ArmSubsystem.ARM_TARGET_POSITION ARM_POSITION){
        armSubsystem.update(ARM_POSITION);
    }
}
