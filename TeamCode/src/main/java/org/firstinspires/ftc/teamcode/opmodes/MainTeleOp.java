package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.hardware.Constants.planeLaunchPosition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.SetArmPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SetClawAngleCommand;
import org.firstinspires.ftc.teamcode.commands.SetClawStateCommand;
import org.firstinspires.ftc.teamcode.hardware.CalibrationTransfer;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;


@Config
@TeleOp(name = "😇")


public class MainTeleOp extends CommandOpMode {
    RobotHardware hardware;
    DrivetrainSubsystem swerve;
    CalibrationTransfer file;
    ArmSubsystem armSubsystem;
    GamePad gamePad;

    boolean isOpModeStarted = false;




    @Override
    public void initialize() {
        hardware = new RobotHardware(hardwareMap);
        hardware.initialiseHardware(telemetry);
        hardware.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        armSubsystem = new ArmSubsystem(hardware,telemetry,false);
        file = new CalibrationTransfer(telemetry);


        telemetry.clearAll();
        telemetry.addLine("press triangle 🔺 if modules calibrated");
        telemetry.addLine("press cross ❌ if read angles from file");
        telemetry.update();


        while (!this.isStopRequested()) {
            if (gamepad1.triangle) {
                swerve = new DrivetrainSubsystem(hardware, telemetry, false);
                telemetry.addLine("initialising standard teleOp");
                telemetry.addLine("waiting for start ✅");
                break;
            } else if (gamepad1.cross) {
                double [] angles = file.pullModuleAngleOffsets();
                double heading = file.getRobotHeadingOffset();
                telemetry.addData("0", angles[0]);
                telemetry.addData("1", angles[1]);
                telemetry.addData("2", angles[2]);
                telemetry.addData("heading", heading);

                swerve = new DrivetrainSubsystem(hardware, telemetry, angles, heading, true);
                telemetry.addLine("read from file");
                telemetry.addLine("waiting for start ✅");
                break;
            }
        }
        telemetry.update();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamePad = new GamePad(gamepad1);

        swerve.stopAllMotors();
        swerve.resetAllEncoders();



        gamePad.getGamepadButton(GamepadKeys.Button.B).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.HIGHPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.MIDPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.X).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.PICKUP));
        gamePad.getGamepadButton(GamepadKeys.Button.A).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.LOWPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new SetArmPositionCommand(armSubsystem, ArmSubsystem.ARM_TARGET_POSITION.TRANSFER));
        gamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).toggleWhenPressed(new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_OPEN), new SetClawStateCommand(armSubsystem, ArmSubsystem.CLAW_STATE.BOTH_CLOSED));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).toggleWhenPressed(new SetClawAngleCommand(armSubsystem, ArmSubsystem.CLAW_ANGLE.PICKUP), new SetClawAngleCommand(armSubsystem, ArmSubsystem.CLAW_ANGLE.HIGHPOS));
    }





    @Override
    public void run(){
        hardware.clearBulkCache();
        super.run();


        if (!isOpModeStarted){
            hardware.startIMUThread(this);
            Thread voltageThread = new Thread(hardware);
            voltageThread.start();
            isOpModeStarted = true;
        }
        swerve.drive(gamePad.getJoystickVector().scale(hardware.getDriveMultiplierFromBatteryVoltage()), gamePad.getTurnSpeed());


        if (gamepad1.options) hardware.planeServo.setPosition(planeLaunchPosition);


        if (isStopRequested()) {
            while (!file.hasWrote) file.PushCalibrationData(hardware.imu.getRotation2d().getRadians(), swerve.getAllModuleAngleRads());
        }
        telemetry.update();
    }
}