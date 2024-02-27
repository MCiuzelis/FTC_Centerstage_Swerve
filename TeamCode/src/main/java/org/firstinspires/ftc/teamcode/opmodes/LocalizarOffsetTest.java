package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLaunchPosition;
import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLockPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.DepositPixelsCommand;
import org.firstinspires.ftc.teamcode.commands.SetArmToStateCommand;
import org.firstinspires.ftc.teamcode.commands.lowLevelCommands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.Localizer;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;

@Photon
@TeleOp(name = "LocalizerTest")


public class LocalizarOffsetTest extends CommandOpMode {
    RobotHardware hardware;
    DrivetrainSubsystem swerve;
    CalibrationTransfer file;
    GamePad gamePad;
    Localizer localizer;

    boolean opModeStarted = false;

    double startingDistance = 0;
    double sum = 0;
    double count = 1;




    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        hardware = new RobotHardware(hardwareMap);
        hardware.initialiseHardware(telemetry);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        file = new CalibrationTransfer(telemetry);


        swerve = new DrivetrainSubsystem(hardware, telemetry, false, false);
        swerve.resetImuOffset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamePad = new GamePad(gamepad1);
        localizer = new Localizer(hardware, telemetry, true);
    }





    @Override
    public void run(){
        hardware.clearBulkCache();

        if (!opModeStarted){
            hardware.startIMUThread(this);
            opModeStarted = true;
            startingDistance = hardware.distanceSensor.getDistance(DistanceUnit.CM);
        }

        if (gamepad1.share){
            swerve.resetImuOffset();
        }

        swerve.setGamepadInput(gamePad.getGamepadInput());
        CommandScheduler.getInstance().run();
        swerve.drive();

        localizer.updateOdometry();
        Pose2d robotPosition = localizer.getRobotCentricPosition();
        telemetry.addData("position", robotPosition);

        if (gamepad1.cross){
            count += 1;
            sum += (hardware.distanceSensor.getDistance(DistanceUnit.CM) - startingDistance) / robotPosition.getY();
        }

        telemetry.addData("sum", sum);
        telemetry.addData("count", count);
        telemetry.addData("multiplier", sum / count);
        telemetry.update();


        if (isStopRequested()) while (!file.hasWrote) file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
    }
}