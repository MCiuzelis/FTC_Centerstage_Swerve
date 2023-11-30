package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class UpdateClawStateCommand extends SequentialCommandGroup {
    public UpdateClawStateCommand(ArmSubsystem armSubsystem, ArmSubsystem.CLAW_STATE CLAW_STATE){
        armSubsystem.update(CLAW_STATE);
    }
}