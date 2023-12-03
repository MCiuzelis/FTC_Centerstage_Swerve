package org.firstinspires.ftc.teamcode.commands.AutonomousCommands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DriveTimeoutCommand extends SequentialCommandGroup {
    public DriveTimeoutCommand(DrivetrainSubsystem driveTrain, double speedMultiplier, double angleDegrees, double turnSpeed, long time){

        addCommands(
                new DriveCommand(driveTrain, speedMultiplier, angleDegrees, turnSpeed)
                        .andThen(new WaitCommand(time))
                        .andThen(new InstantCommand(driveTrain::stop))
        );

    }
}