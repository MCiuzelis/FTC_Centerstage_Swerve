package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.AutonomousCommands.*;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.PropDetectionPipeline;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Config
@Autonomous(name = "MainAutonomous", group = "OpMode")
public class MainAutonomous extends CommandOpMode {
    private final RobotHardware robot = new RobotHardware();
    private final CalibrationTransfer file = new CalibrationTransfer(telemetry);
    private DrivetrainSubsystem drivetrain;
    private ArmSubsystem arm;
    OpenCvCamera camera;
    int cameraMonitorViewId;
    PropDetectionPipeline pipeline = new PropDetectionPipeline();
    public boolean activated = false;


    @Override
    public void initialize() {
        robot.init(hardwareMap, telemetry);
        drivetrain = new DrivetrainSubsystem(robot, telemetry, false);
        arm = new ArmSubsystem(robot, telemetry, false);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.webcam, cameraMonitorViewId);
        pipeline = new PropDetectionPipeline();

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened() {camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);}
            @Override
            public void onError(int errorCode) {telemetry.addData("Error: ", errorCode);}
        });
    }



    @Override
    public void run() {
        super.run();

        if (!activated) {
            switch (pipeline.getPositionResult()) {
                case LEFT:
                    schedule(new AutonomousLeftCommand(arm, drivetrain, file, robot));
                    activated = true;
                    break;
                case MIDDLE:
                    schedule(new AutonomousMiddleCommand(arm, drivetrain, file, robot));
                    activated = true;
                    break;
                case RIGHT:
                    schedule(new AutonomousRightCommand(arm, drivetrain, file, robot));
                    activated = true;
                    break;
            }
        }
    }
}