package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.Localizer;

@Photon
@Disabled
@TeleOp(name = "DEBUG_TRACKBALL")


public class DebugTrackball extends CommandOpMode {
    RobotHardware hardware;
    Localizer localizer;
    DrivetrainSubsystem swerve;
    CalibrationTransfer file;
    GamePad gamePad;
    boolean opModeStarted = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        hardware = new RobotHardware(hardwareMap);
        hardware.initialiseHardware(telemetry);
        localizer = new Localizer(hardware, telemetry, true);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);


        telemetry.clearAll();
        telemetry.addLine("press triangle 🔺 if modules calibrated");
        telemetry.addLine("press cross ❌ if read angles from file");
        telemetry.update();


        while (!this.isStopRequested()) {
            if (gamepad1.triangle) {
                swerve = new DrivetrainSubsystem(hardware, telemetry, false, true);
                telemetry.addLine("initialising standard teleOp");
                telemetry.addLine("waiting for start ✅");
                swerve.resetAllEncoders();
                break;
            } else if (gamepad1.cross) {
                double [] angles = file.pullModuleAngleOffsets();
                double heading = file.getRobotHeadingOffset();
                telemetry.addData("0", angles[0]);
                telemetry.addData("1", angles[1]);
                telemetry.addData("2", angles[2]);
                telemetry.addData("heading", heading);


                swerve = new DrivetrainSubsystem(hardware, telemetry, heading, false, true);
                telemetry.addLine("read from file");
                telemetry.addLine("waiting for start ✅");
                break;
            }
        }
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamePad = new GamePad(gamepad1);
    }


    @Override
    public void run(){
        if (!opModeStarted){
            hardware.startIMUThread(this);
            opModeStarted = true;
        }

        hardware.clearBulkCache();
        swerve.setGamepadInput(gamePad.getGamepadInput());
        swerve.drive();

        localizer.updateOdometry();

        Pose2d position = localizer.getRobotCentricPosition();
        Pose2d velocity = localizer.getRobotCentricVelocity();

        telemetry.addData("X", position.getX());
        telemetry.addData("Y", position.getY());
        telemetry.addData("Z", Math.toDegrees(position.getHeading()));

        telemetry.addData("robotVelocity", velocity);

//        double[] raw = localizer.getRaw();
//        telemetry.addData("0", raw[0]);
//        telemetry.addData("1", raw[1]);
//        telemetry.addData("2", raw[2]);
//        telemetry.addData("3", raw[3]);

        telemetry.update();
    }
}