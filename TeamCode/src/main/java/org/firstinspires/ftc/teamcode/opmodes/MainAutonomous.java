package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.ResetDriveEncodersCommand;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

@Autonomous(name = "MainAutonomous", group = "OpMode")
public class MainAutonomous extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    ElapsedTime timer = new ElapsedTime();



    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        timer.reset();
    }

    @Override
    public void run(){
        super.run();

        while (timer.seconds() < 5) {
            robot.intakeBroomMotor.setPower(1);
        }
        robot.intakeBroomMotor.setPower(0);
    }
}