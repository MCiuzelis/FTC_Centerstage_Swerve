package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.InstantCommands.*;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.ReleasePixelCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToDepositPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToPickupPositionCommand;
import org.firstinspires.ftc.teamcode.commands.SequentialCommands.SetArmToTransferPositionCommand;
import org.firstinspires.ftc.teamcode.hardware.GamePad;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.*;

@Config
@TeleOp(name = "MainTeleOpMode", group = "OpMode")
public class MainTeleOpMode extends CommandOpMode {
    private final RobotHardware robot = RobotHardware.getInstance();
    DrivetrainSubsystem drivetrainSubsystem;
    GamePad gamePad;


    @Override
    public void initialize() {

        robot.init(hardwareMap, telemetry);
        drivetrainSubsystem = new DrivetrainSubsystem(robot, telemetry, true);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamePad = new GamePad(gamepad1);
        ArmSubsystem arm = new ArmSubsystem(robot, telemetry, false);
        PlaneLauncherSubsystem plane = new PlaneLauncherSubsystem(robot, telemetry);

        drivetrainSubsystem.ResetAllEncoders();

        plane.update(PlaneLauncherSubsystem.PLANE_STATE.LOCK);

//        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_UP)
//                .whenPressed(new SetArmToPositionCommand(arm, ArmSubsystem.ARM_TARGET_POSITION.PICKUP));
//        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
//                .whenPressed(new SetArmToPositionCommand(arm, ArmSubsystem.ARM_TARGET_POSITION.DEPOSIT));


        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_OPEN));
        gamePad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new UpdateClawStateCommand(arm, ArmSubsystem.CLAW_STATE.BOTH_CLOSED));


        gamePad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new SetArmToPickupPositionCommand(arm));
        gamePad.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new SetArmToTransferPositionCommand(arm));
        gamePad.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new SetArmToDepositPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.MIDPOS));
        gamePad.getGamepadButton(GamepadKeys.Button.X)
                .toggleWhenPressed(new SetArmToDepositPositionCommand(arm, ArmSubsystem.SLIDE_TARGET_POSITION.HIGHPOS));

        gamePad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                        .whenPressed(new InstantCommand(()-> plane.update(PlaneLauncherSubsystem.PLANE_STATE.LAUNCH)));


        gamePad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new ReleasePixelCommand(arm));
    }

    @Override
    public void run(){
        super.run();
        drivetrainSubsystem.Drive(gamePad.getJoystickVector(), gamePad.getTurnSpeed());

        telemetry.addData("armMotor", robot.armMotor.getCurrentPosition());
        telemetry.addData("liftMotor", robot.slideMotor.getCurrentPosition());
        telemetry.update();


        /*
        if(gamePad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9){
           schedule(new SetPlaneStateCommand(plane, PlaneLauncherSubsystem.PLANE_STATE.LAUNCH));
        }
        else schedule(new SetPlaneStateCommand(plane, PlaneLauncherSubsystem.PLANE_STATE.LOCK));

         */
    }
}