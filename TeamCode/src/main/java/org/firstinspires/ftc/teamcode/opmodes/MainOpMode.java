package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ResetDriveEncodersCommand;
import org.firstinspires.ftc.teamcode.commands.SetDepositorPinCommand;
import org.firstinspires.ftc.teamcode.commands.SetDepositorProbePositionCommand;
import org.firstinspires.ftc.teamcode.commands.SetIntakeBroomCommand;
import org.firstinspires.ftc.teamcode.commands.SetIntakeHingePositionCommand;
import org.firstinspires.ftc.teamcode.commands.SetLiftPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TransferCommand;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DepositorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@TeleOp(name = "MainTeleOpMode", group = "OpMode")
public class MainOpMode extends CommandOpMode {
    public IntakeSubsystem intakeSubsystem;
    public DepositorSubsystem depositorSubsystem;
    public DrivetrainSubsystem drivetrainSubsystem;
    GamePad gamePad;
    private final RobotHardware robot = RobotHardware.getInstance();



    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(robot, telemetry, true);
        depositorSubsystem = new DepositorSubsystem(robot, telemetry, true);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);

        new ResetDriveEncodersCommand(drivetrainSubsystem);

        gamePad = new GamePad(gamepad1);

        //region BindTriggers ->
        //Toggle depositor pin -> Left Bumper
        gamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.RELEASE), new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.HOLD));
        //Lower linear slides
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whileHeld(() -> new SetLiftPositionCommand(depositorSubsystem, 0.5f, false)).whenReleased(() -> schedule(new InstantCommand(depositorSubsystem::StopSlideMotor, depositorSubsystem)));
        //Raise linear slides
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(() -> new SetLiftPositionCommand(depositorSubsystem, 0.5f, true)).whenReleased(() -> schedule(new InstantCommand(depositorSubsystem::StopSlideMotor, depositorSubsystem)));
        //Toggle depositor probe: DPAD_LEFT -> low | DPAD_RIGHT -> high
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(() -> schedule(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.DEPOSIT)));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(() -> schedule(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.PICKUP)));
        //toggle broom rotation -> RIGHT_BUMPER
        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new SetIntakeBroomCommand(intakeSubsystem, IntakeSubsystem.BROOM_MOTOR_STATE.ACTIVE), new SetIntakeBroomCommand(intakeSubsystem, IntakeSubsystem.BROOM_MOTOR_STATE.REVERSED));
        //stop broom rotation -> RIGHT_BUMPER + TRIANGLE
        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(new GamepadButton(gamePad, GamepadKeys.Button.X)).whenActive(new SetIntakeBroomCommand(intakeSubsystem, IntakeSubsystem.BROOM_MOTOR_STATE.DISABLED));
        //Raise/lower intake -> CROSS
        gamePad.getGamepadButton(GamepadKeys.Button.B)
                .toggleWhenPressed(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.PICKUP), new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.TRANSFER));

        gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.LOWPOS)));

        gamePad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.HIGHPOS)));

        gamePad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new TransferCommand(intakeSubsystem, depositorSubsystem)));

        //endregion
    }

    @Override
    public void run(){
        super.run();
        drivetrainSubsystem.Drive(gamePad);
    }
}
