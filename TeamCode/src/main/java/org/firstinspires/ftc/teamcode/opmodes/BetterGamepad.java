package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

public class BetterGamepad extends GamepadEx {
    public static final GamepadKeys.Button circle = GamepadKeys.Button.B;
    public static final GamepadKeys.Button cross = GamepadKeys.Button.A;
    public static final GamepadKeys.Button triangle = GamepadKeys.Button.Y;
    public static final GamepadKeys.Button square = GamepadKeys.Button.X;
    public static final GamepadKeys.Button share = GamepadKeys.Button.BACK;
    public static final GamepadKeys.Button options = GamepadKeys.Button.START;

    private Gamepad gamepad;

    public BetterGamepad(Gamepad gamepad) {
        super(gamepad);
        this.gamepad = gamepad;
    }

    public void rumble (int durationMs){
        gamepad.rumble(durationMs);
    }

    public void rumble (double rumble1, double rumble2, int durationMs){
        gamepad.rumble(rumble1, rumble2, durationMs);
    }

    public float leftTrigger(){
        return gamepad.left_trigger;
    }

    public float rightTrigger(){
        return gamepad.right_trigger;
    }

    public Pose2d getGamepadInput(double driveScalar, double turnScalar){
        double left_stick_x = rootInput(gamepad.left_stick_x);
        double left_stick_y = rootInput(gamepad.left_stick_y);
        Vector2d drive = new Vector2d(left_stick_x, left_stick_y);

        double PositiveDriveAngle = Math.abs(drive.angle());
        double maximumNotScaledDownSpeedInARectangularContour = (PositiveDriveAngle >= Math.toRadians(45) && PositiveDriveAngle <= Math.toRadians(135)) ? Math.hypot(left_stick_x, 1) : Math.hypot(left_stick_y, 1);

        drive = drive.div(maximumNotScaledDownSpeedInARectangularContour);
        drive = drive.scale(drive.magnitude());

        double turn = -gamepad.right_stick_x;

        if (gamepad.right_trigger > 0.1) {
            drive = drive.scale(0.1);
            turn *= 0.35;
        }
        else if (gamepad.left_stick_x < 0.1) {
            drive = drive.scale(0.8);
            turn *= 0.8;
        }

        drive = drive.scale(driveScalar);
        turn *= turnScalar;
        return new Pose2d(drive.getX(), drive.getY(), turn);
    }

    public Pose2d getGamepadInput(){
        return getGamepadInput(1, 1);
    }
    public Pose2d getGamepadInput(double driveScalar){
        return getGamepadInput(driveScalar, 1);
    }
    public float squareInput(double input) {return (float) (-Math.signum(input) * Math.pow(input, 2));}
    public float rootInput(double input) {return (float) (-Math.signum(input) * Math.sqrt(Math.abs(input)));}
}
