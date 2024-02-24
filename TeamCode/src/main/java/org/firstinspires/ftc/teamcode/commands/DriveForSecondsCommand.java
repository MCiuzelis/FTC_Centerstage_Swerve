package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;


public class DriveForSecondsCommand extends CommandBase {

    Pose2d driveInput;
    DrivetrainSubsystem robot;
    double time;
    ElapsedTime timer;

    public DriveForSecondsCommand(DrivetrainSubsystem robot, Pose2d driveInput, double time){
        this.driveInput = driveInput;
        this.robot = robot;
        this.time = time;
        timer = new ElapsedTime();
        addRequirements(robot);
    }

    public DriveForSecondsCommand(DrivetrainSubsystem robot, Vector2d driveInput, double time){
        this(robot, new Pose2d(driveInput, 0), time);
    }


    @Override
    public void execute(){
        robot.setGamepadInput(driveInput);
    }

    @Override
    public boolean isFinished(){
        return timer.seconds() >= time;
    }


    @Override
    public void end (boolean interrupted){
        robot.setGamepadInput();
    }
}
