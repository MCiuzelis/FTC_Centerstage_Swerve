package org.firstinspires.ftc.teamcode.commands.InstantCommands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class ReleasePixels extends InstantCommand {
    public ReleasePixels(ArmSubsystem armSubsystem){
        super(()-> armSubsystem.releasePixel(60));
    }
}
