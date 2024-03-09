package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLaunchPosition;
import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLockPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.DepositPixelsCommand;
import org.firstinspires.ftc.teamcode.commands.SetArmToStateCommand;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.utils.CalibrationTransfer;

@Photon
@TeleOp(name = "😇")


public class MainTeleOp extends CommandOpMode {
    RobotHardware hardware;
    DrivetrainSubsystem swerve;
    CalibrationTransfer file;
    ArmSubsystem armSubsystem;
    GamePad gamePad;

    double loopTime = 0;
    boolean opModeStarted = false;


    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        hardware = new RobotHardware(hardwareMap);
        hardware.initialiseHardware(telemetry);

        armSubsystem = new ArmSubsystem(hardware,telemetry,true);
        file = new CalibrationTransfer(telemetry);


        telemetry.clearAll();
        telemetry.addLine("press triangle 🔺 if modules calibrated");
        telemetry.addLine("press cross ❌ if read angles from file");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //localizer = new Localizer(hardware, telemetry, true);
        gamePad = new GamePad(gamepad1);


        gamePad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(()-> schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.HIGH)));
        gamePad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(()-> schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER)));
        gamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(()-> schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.MID)));
        gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(()-> schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.LOW)));
        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(()-> schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.PICKUP)));
        gamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(()-> schedule(new InstantCommand(()-> armSubsystem.invertClawState())));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(()-> schedule(new DepositPixelsCommand(armSubsystem, swerve, 0.25)));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(()-> schedule(new DepositPixelsCommand(armSubsystem, swerve, -0.25)));
        gamePad.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(()-> schedule(new InstantCommand(()-> hardware.planeServo.setPosition(planeLaunchPosition))),
                                   ()-> schedule(new InstantCommand(()-> hardware.planeServo.setPosition(planeLockPosition))));


        while (!this.isStopRequested()) {
            if (gamepad1.triangle) {
                swerve = new DrivetrainSubsystem(hardware, telemetry, false, false);
                telemetry.addLine("initialising standard teleOp");
                telemetry.addLine("waiting for start ✅");
                swerve.calibrate();
                while (!swerve.areModulesCalibrated()){
                    sleep(1);
                    swerve.updateModuleAngles();
                }
                swerve.resetAllEncoders();
                swerve.resetImuOffset();
                break;
            } else if (gamepad1.cross) {
                double [] angles = file.pullModuleAngleOffsets();
                double heading = file.getRobotHeadingOffset();
                telemetry.addData("0", angles[0]);
                telemetry.addData("1", angles[1]);
                telemetry.addData("2", angles[2]);
                telemetry.addData("heading", heading);


                swerve = new DrivetrainSubsystem(hardware, telemetry, heading, false, false);
                telemetry.addLine("read from file");
                telemetry.addLine("waiting for start ✅");
                break;
            }
        }
        telemetry.update();
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }





    @Override
    public void run(){
        if (!opModeStarted){
            hardware.startIMUThread(this);
            opModeStarted = true;
        }
        if (gamepad1.share) swerve.resetImuOffset();


        hardware.clearBulkCache();
        swerve.updateModuleAngles();

        double driveScalar = armSubsystem.isArmUp ? 0.25 : 1;

        swerve.setGamepadInput(gamePad.getGamepadInput(driveScalar));
        CommandScheduler.getInstance().run();
        swerve.drive();


        if (armSubsystem.rumble){
            gamepad1.rumble(250);
            //telemetry.speak("Matas is pretty monkey");
            armSubsystem.rumble = false;
        }

        double loop = System.nanoTime();
        telemetry.addData("rumble: ", armSubsystem.rumble);
        telemetry.addData("Loop time ms",  (loop - loopTime) / 1000000);
        telemetry.addData("imuAngle", hardware.imuAngle.getDegrees());
        loopTime = loop;
        telemetry.addData("time left: ", 120 - getRuntime());

        swerve.updateChassisSpeedFromEncoders();
        Vector2d chassisVector = swerve.getRobotXYVelocity();
        Pose2d robotPosition = swerve.getRobotPosition();

        telemetry.addData("chassis speed X: ", chassisVector.getX());
        telemetry.addData("chassis speed Y: ", chassisVector.getY());
        telemetry.addData("position X: ", robotPosition.getX());
        telemetry.addData("position Y: ", robotPosition.getY());
        telemetry.update();


//        if (getRuntime() > 121){
//            CommandScheduler.getInstance().schedule(new SetArmToStateCommand(armSubsystem, SetArmToStateCommand.ArmState.TRANSFER));
//            CommandScheduler.getInstance().run();
//            while (!file.hasWrote) file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
//            sleep(100);
//            requestOpModeStop();
//        }
    }
}