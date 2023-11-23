package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;

public class SetDepositorPinCommand extends InstantCommand {
    public SetDepositorPinCommand(DepositorSubsystem depositor, DepositorSubsystem.DEPOSITOR_PIN_STATE state){
        super(()->depositor.update(state));

    }
}