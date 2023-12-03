package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import static org.firstinspires.ftc.teamcode.hardware.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class PlaneLauncherSubsystem {
    public RobotHardware robot;
    public Telemetry telemetry;

    public PlaneLauncherSubsystem(RobotHardware robot, Telemetry telemetry){
        this.robot = robot;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }


    public void update(PLANE_STATE state){
        switch (state){
            case LAUNCH:
                robot.planeServo.setPosition(planeLaunchPosition);
                break;
            case LOCK:
                robot.planeServo.setPosition(planeLockPosition);
                break;
        }
    }

    public enum PLANE_STATE{
        LAUNCH,
        LOCK
    }
}
