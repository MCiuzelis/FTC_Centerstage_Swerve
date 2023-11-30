package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncherSubsystem;

public class SetPlaneStateCommand extends SequentialCommandGroup {
    public SetPlaneStateCommand(PlaneLauncherSubsystem planeLauncherSubsystem, PlaneLauncherSubsystem.PLANE_STATE STATE){
        planeLauncherSubsystem.update(STATE);
    }
}
