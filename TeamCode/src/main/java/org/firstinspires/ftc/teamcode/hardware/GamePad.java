package org.firstinspires.ftc.teamcode.hardware;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamePad extends GamepadEx {

    public GamePad(Gamepad gamepad) {
        super(gamepad);
    }


    public Pose2d getGamepadInput(){
        double left_stick_y = super.getLeftY();
        double left_stick_x = super.getLeftX();

        Vector2d drive = new Vector2d(left_stick_x, left_stick_y);

        double PositiveDriveAngle = Math.abs(drive.angle());
        double maximumNotScaledDownSpeedInARectangularContour = (PositiveDriveAngle >= Math.toRadians(45) && PositiveDriveAngle <= Math.toRadians(135)) ? Math.hypot(left_stick_x, 1) : Math.hypot(left_stick_y, 1);

        drive = drive.div(maximumNotScaledDownSpeedInARectangularContour);
        drive = drive.scale(drive.magnitude());

        double turn = -super.getRightX();

        if (gamepad.right_trigger > 0.9){
            drive = drive.scale(0.1);
            turn *= 0.5;
        }

        return new Pose2d(drive.getX(), drive.getY(), new Rotation2d(turn));
    }



    private double squareInput(double input){
        return -Math.signum(input) * Math.pow(input, 2);
    }
}