package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLaunchPosition;
import static org.firstinspires.ftc.teamcode.hardware.Globals.planeLockPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

        armSubsystem = new ArmSubsystem(hardware,telemetry,false);
        file = new CalibrationTransfer(telemetry);


        telemetry.clearAll();
        telemetry.addLine("press triangle 🔺 if modules calibrated");
        telemetry.addLine("press cross ❌ if read angles from file");
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
                .toggleWhenPressed(()-> schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_OPEN)),
                                   ()-> schedule(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED)));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                        .whenPressed(()-> schedule(new DepositPixelsCommand(armSubsystem, swerve)));
//        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whileHeld(()-> schedule(new InstantCommand(()-> swerve.setModeToMaintainDistance(Math.toRadians(90), 12))))
//                .whenReleased(()-> schedule(new InstantCommand(()-> swerve.setModeToManual())));
        gamePad.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(()-> schedule(new InstantCommand(()-> hardware.planeServo.setPosition(planeLaunchPosition))),
                                   ()-> schedule(new InstantCommand(()-> hardware.planeServo.setPosition(planeLockPosition))));


        while (!this.isStopRequested()) {
            if (gamepad1.triangle) {
                swerve = new DrivetrainSubsystem(hardware, telemetry, false, false);
                telemetry.addLine("initialising standard teleOp");
                telemetry.addLine("waiting for start ✅");
                swerve.calibrate();
                while (!swerve.areModulesCalibrated()){sleep(5);}
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

        if (gamepad1.share){
            swerve.resetImuOffset();
        }


        hardware.clearBulkCache();
        swerve.setGamepadInput(gamePad.getGamepadInput());
        CommandScheduler.getInstance().run();
        swerve.drive();

        telemetry.addData("distance", hardware.distanceSensor.getDistance(DistanceUnit.CM));

        double loop = System.nanoTime();
        telemetry.addData("Loop time ms",  (loop - loopTime) / 1000000);
        telemetry.addData("imuAngle", hardware.imuAngle.getDegrees());
        loopTime = loop;

        telemetry.addData("time left: ", 120 - getRuntime());
        telemetry.update();

        //if (getRuntime() > 120) requestOpModeStop();
        if (isStopRequested()) while (!file.hasWrote) file.PushCalibrationData(hardware.imuAngle.getRadians(), swerve.getAllModuleAngleRads());
    }
}