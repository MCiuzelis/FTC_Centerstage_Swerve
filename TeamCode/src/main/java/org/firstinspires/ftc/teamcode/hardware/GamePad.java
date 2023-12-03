package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePad extends GamepadEx {

    public GamePad(Gamepad gamepad) {
        super(gamepad);
    }

    public double getTurnSpeed(){
        double rightY = super.getRightX();
        if (super.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            rightY *= 0.15;
        }
        return -Math.signum(rightY) * Math.pow(rightY, 2);
    }

    public Vector2d getJoystickVector(){
        double left_stick_y = super.getLeftY();
        double left_stick_x = super.getLeftX();

        Vector2d vector2d = new Vector2d(left_stick_x, left_stick_y);

        double PositiveDriveAngle = Math.abs(vector2d.angle());
        double maximumNotScaledDownSpeedInARectangularContour = (PositiveDriveAngle >= Math.toRadians(45) && PositiveDriveAngle <= Math.toRadians(135)) ? Math.hypot(left_stick_x, 1) : Math.hypot(left_stick_y, 1);

        vector2d = vector2d.div(maximumNotScaledDownSpeedInARectangularContour);
        vector2d = vector2d.scale(vector2d.magnitude());

        if (super.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            vector2d = vector2d.scale(0.25);
        }

        return vector2d;
    }
}