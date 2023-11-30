package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class SetSlidesToPositionCommand extends SequentialCommandGroup {
    public SetSlidesToPositionCommand(ArmSubsystem armSubsystem, ArmSubsystem.SLIDE_TARGET_POSITION LIFT_POSITION){
        armSubsystem.update(LIFT_POSITION);
    }
}
