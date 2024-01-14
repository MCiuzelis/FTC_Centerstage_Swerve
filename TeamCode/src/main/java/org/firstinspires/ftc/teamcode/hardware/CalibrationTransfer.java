package org.firstinspires.ftc.teamcode.hardware;

import android.os.Environment;

import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Scanner;

public class CalibrationTransfer {

    private final Telemetry telemetry;
    private final String path = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/calibration.txt";
    private final List<String> calibrationList = new ArrayList<>();
    public boolean hasWrote= false;
    private String[] array;
    public CalibrationTransfer(Telemetry telemetry){
        this.telemetry = telemetry;
    }



    public void PushCalibrationData(double robotAngle, double[] moduleAngles) {
        try {
            FileWriter myWriter = new FileWriter(path);
            myWriter.write(robotAngle + "\n" + moduleAngles[0] + "\n" + moduleAngles[1] + "\n" + moduleAngles[2]);
            myWriter.close();
            hasWrote=true;
            //telemetry.addLine("Calibration saved!");
            //telemetry.update();
        } catch (IOException e) {
            telemetry.addLine("Fatal error writing to file!");
            telemetry.update();
            e.printStackTrace();
        }
    }



    public double[] pullModuleAngleOffsets(){
        try{
            File myObj = new File(path);
            Scanner myReader = new Scanner(myObj);
            while (myReader.hasNextLine()) {
                String data = myReader.nextLine();

                calibrationList.add(data);

                //telemetry.addData("Reading calibration file", data);
                //telemetry.update();
            }
            myObj.delete();
            myReader.close();
        } catch (FileNotFoundException e) {
            telemetry.addLine("Fatal error reading from file!");
            telemetry.update();
            e.printStackTrace();
        }

        array = calibrationList.toArray(new String[0]);
        return new double[] {Double.parseDouble(array[1]), Double.parseDouble(array[2]), Double.parseDouble(array[3])};
    }



    public double getRobotHeadingOffset(){
        return (Double.parseDouble(array[0]));
    }
}