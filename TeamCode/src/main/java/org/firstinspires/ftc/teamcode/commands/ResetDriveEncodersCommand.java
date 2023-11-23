package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class ResetDriveEncodersCommand extends InstantCommand {
    public ResetDriveEncodersCommand(DrivetrainSubsystem drive){
        super(() -> drive.ResetAllEncoders());
    }
}
