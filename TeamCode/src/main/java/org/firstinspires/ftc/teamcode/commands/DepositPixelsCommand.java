package org.firstinspires.ftc.teamcode.commands;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public class DepositPixelsCommand extends SequentialCommandGroup {
    public DepositPixelsCommand(ArmSubsystem arm, DrivetrainSubsystem robot, double X_AxisDriveSpeed){

        addCommands(
                new SetClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN),
                new WaitCommand(50),
                new InstantCommand(()-> arm.offsetLift(-50)),
                new WaitCommand(30),
                new DriveForSecondsCommand(robot, new Vector2d(X_AxisDriveSpeed, 0), 0.4),
                new SetArmToStateCommand(arm, SetArmToStateCommand.ArmState.TRANSFER)
        );
        addRequirements(arm, robot);
    }
}