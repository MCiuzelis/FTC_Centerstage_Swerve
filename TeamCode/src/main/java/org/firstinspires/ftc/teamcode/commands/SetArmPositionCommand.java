package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetArmPositionCommand extends InstantCommand {
    public SetArmPositionCommand(ArmSubsystem arm, ArmSubsystem.ARM_TARGET_POSITION targetPos){
        super(() -> arm.update(targetPos));
    }
}
