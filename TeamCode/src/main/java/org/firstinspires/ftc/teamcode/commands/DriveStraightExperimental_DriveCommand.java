package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;


public class DriveStraightExperimental_DriveCommand extends CommandBase {

    DrivetrainSubsystem robot;

    public DriveStraightExperimental_DriveCommand(DrivetrainSubsystem robot) {
        this.robot = robot;
        addRequirements(robot);
    }


    @Override
    public void execute() {
        Pose2d joystickInput = robot.getGamepadInput();
        double joystickAngle = Math.abs(Math.atan2(joystickInput.getY(), joystickInput.getX()));
        double magnitude = Math.hypot(joystickInput.getX(), joystickInput.getY());


        Vector2d drive;
        if (joystickAngle > Math.toRadians(45) && joystickAngle < Math.toRadians(135)) {
            drive = new Vector2d(0, magnitude * Math.signum(joystickInput.getY()));
        } else {
            drive = new Vector2d(magnitude * Math.signum(joystickInput.getX()), 0);
        }
        robot.setGamepadInput(new Pose2d(drive, joystickInput.getHeading()));
    }
}