package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;

public abstract class TeleOpBase extends CommandOpMode {
    public RobotHardware robot;
    public BetterGamepad driver;
    public DrivetrainSubsystem swerve;

    private boolean start = false;

    @Override
    public void initialize(){
        robot = new RobotHardware(hardwareMap);
        robot.initialiseHardware(telemetry);

        driver = new BetterGamepad(gamepad1);

        Init();
        while (opModeInInit()) init_loop();
    }

    @Override
    public void run(){
        if (!start){
            Start();
            start = true;
        }
        swerve.periodic();
        Loop();

        if (isStopRequested()){
            Stop();
            stop();
        }
    }

    public abstract void Init();
    public abstract void InitLoop();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();
}