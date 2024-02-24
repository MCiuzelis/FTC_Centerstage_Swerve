package org.firstinspires.ftc.teamcode.hardware;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePad extends GamepadEx {

    public GamePad(Gamepad gamepad) {
        super(gamepad);
    }


    public Pose2d getGamepadInput(){
        double left_stick_x = rootInput(gamepad.left_stick_x);
        double left_stick_y = rootInput(gamepad.left_stick_y);
        Vector2d drive = new Vector2d(left_stick_x, left_stick_y);

        double PositiveDriveAngle = Math.abs(drive.angle());
        double maximumNotScaledDownSpeedInARectangularContour = (PositiveDriveAngle >= Math.toRadians(45) && PositiveDriveAngle <= Math.toRadians(135)) ? Math.hypot(left_stick_x, 1) : Math.hypot(left_stick_y, 1);

        drive = drive.div(maximumNotScaledDownSpeedInARectangularContour);
        drive = drive.scale(drive.magnitude());

        double turn = -gamepad.right_stick_x;

        if (gamepad.left_trigger > 0.1) {
            drive = drive.scale(0.1);
            turn *= 0.35;
        }
        else if (gamepad.right_trigger < 0.1) {
            drive = drive.scale(0.7);
            turn *= 0.6;
        }

        return new Pose2d(drive.getX(), drive.getY(), turn);
    }



    private double squareInput(double input) {return -Math.signum(input) * Math.pow(input, 2);}
    private double rootInput(double input) {return -Math.signum(input) * Math.sqrt(Math.abs(input));}
}