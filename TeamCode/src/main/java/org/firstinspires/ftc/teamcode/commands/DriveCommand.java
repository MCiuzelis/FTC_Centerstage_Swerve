package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;


public class DriveCommand extends CommandBase {

    Pose2d driveInput;
    DrivetrainSubsystem robot;
    double time;
    ElapsedTime timer;

    public DriveCommand (DrivetrainSubsystem robot, Pose2d driveInput, double time){
        this.driveInput = driveInput;
        this.robot = robot;
        this.time = time;
        timer = new ElapsedTime();
        addRequirements(robot);
    }


    public DriveCommand (DrivetrainSubsystem robot, Translation2d driveInput, double time){
        this(robot, new Pose2d(driveInput, new Rotation2d()), time);
    }


    @Override
    public void execute(){
        robot.drive(driveInput);
    }

    @Override
    public boolean isFinished(){
        return timer.seconds() >= time;
    }


    @Override
    public void end (boolean interrupted){
        robot.drive();
    }

}
