package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncherSubsystem;

public class SetPlaneStateCommand extends InstantCommand {
    public SetPlaneStateCommand(PlaneLauncherSubsystem planeLauncherSubsystem, PlaneLauncherSubsystem.PLANE_STATE STATE){
        super(()-> planeLauncherSubsystem.update(STATE));
    }
}
