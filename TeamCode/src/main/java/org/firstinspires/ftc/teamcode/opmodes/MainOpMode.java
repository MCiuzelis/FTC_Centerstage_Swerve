package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.*;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Config
@TeleOp(name = "MainTeleOpMode", group = "OpMode")
public class MainOpMode extends CommandOpMode {
    public DrivetrainSubsystem drivetrainSubsystem;
    GamePad gamePad;
    private final RobotHardware robot = RobotHardware.getInstance();


    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drivetrainSubsystem.ResetAllEncoders();
        gamePad = new GamePad(gamepad1);

        //region BindTriggers ->
        //Toggle depositor pin -> Left Bumper

        /*
        gamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .toggleWhenPressed(new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.RELEASE), new SetDepositorPinCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PIN_STATE.HOLD));
        //Lower linear slides
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(() -> new SetLiftPositionCommand(depositorSubsystem, 100));
        //Raise linear slides
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> new SetLiftPositionCommand(depositorSubsystem, -100));

        //Toggle depositor probe: DPAD_LEFT -> low | DPAD_RIGHT -> high
//        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
//                .whenPressed(() -> schedule(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.DEPOSIT)));
//        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
//                .whenPressed(() -> schedule(new SetDepositorProbePositionCommand(depositorSubsystem, DepositorSubsystem.DEPOSITOR_PROBE_STATE.PICKUP)));
//        //toggle broom rotation -> RIGHT_BUMPER
        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .toggleWhenPressed(new LowerRaiseIntakeCommand(intakeSubsystem, false), new LowerRaiseIntakeCommand(intakeSubsystem, true));
//        //stop broom rotation -> RIGHT_BUMPER + TRIANGLE
//        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).and(new GamepadButton(gamePad, GamepadKeys.Button.X)).whenActive(new SetIntakeBroomCommand(intakeSubsystem, IntakeSubsystem.BROOM_MOTOR_STATE.DISABLED));
//        //Raise/lower intake -> CROSS
//        gamePad.getGamepadButton(GamepadKeys.Button.B)
//                .toggleWhenPressed(new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.PICKUP), new SetIntakeHingePositionCommand(intakeSubsystem, IntakeSubsystem.HINGE_SERVO_TARGET_STATE.TRANSFER));



        //gamePad.getGamepadButton(GamepadKeys.Button.A)
        //        .whenPressed(() -> schedule(new SetLiftPositionCommand(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.TRANSFERPOS)));
        gamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> schedule(new SetToDepositingPosition(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.LOWPOS)));
        gamePad.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> schedule(new SetToDepositingPosition(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.MIDPOS)));
        gamePad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> schedule(new SetToDepositingPosition(depositorSubsystem, DepositorSubsystem.SLIDE_TARGET_POSITION.HIGHPOS)));

//        gamePad.getGamepadButton(GamepadKeys.Button.Y)
//                .whenPressed(() -> schedule(new SetToDepositingPosition(depositorSubsystem)));

        gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> schedule(new TransferCommand(intakeSubsystem, depositorSubsystem)));

         */


        //endregion
    }

    @Override
    public void run(){
        super.run();
        drivetrainSubsystem.Drive(gamePad.getJoystickVector(), gamePad.getTurnSpeed());
        telemetry.update();
//        telemetry.addData("lol", gamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));
//        telemetry.update();
//        if(gamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9){
//           schedule(new InstantCommand(()-> robot.planeServo.setPosition(planeLaunchPosition)));
//        }
//        else schedule(new InstantCommand(()-> robot.planeServo.setPosition(planeLockPosition)));
//
//
//        if(gamePad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.9){
//            robot.intakeBroomMotor.setPower(1);
//        }
    }
}
