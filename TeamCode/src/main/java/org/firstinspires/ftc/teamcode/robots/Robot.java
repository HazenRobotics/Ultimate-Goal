package org.firstinspires.ftc.teamcode.robots;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.utils.TensorFlow;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalization;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * This class sets up and manages a robot
 */
public abstract class Robot {

    HardwareMap hardwareMap;
    OpMode opMode;
    Telemetry telemetry;

    // time
    double previousTime;

    // drive
    public Drive driveTrain;

    private static final String DEFAULT_LOG_FILE_NAME = "*robotLog.txt";

    /**
     * Creates a Robot
     * @param hw robot's hardware map
     */
    public Robot( HardwareMap hw, OpMode op ) {
        this.hardwareMap = hw;
        this.opMode = op;
        telemetry = opMode.telemetry;

        //vuforiaKey = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);

        //Bulk Caching to decrease cycle times
        for (LynxModule module : hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /**
     * writes to the default file *robotLog.txt
     * @param writeText what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
     * @param isAppending true: will append to the file if it exists, false: will create a new file
     * @param includeTimeStamp will include the timeStamp for when the method is called
     */
    public static void writeToDefaultFile( String writeText, boolean isAppending, boolean includeTimeStamp ) {
        Log.e( "|-|-|-|", writeText );
        writeAFile( DEFAULT_LOG_FILE_NAME, writeText, isAppending, includeTimeStamp );
    }

    /**
     *
     * @param fileName the name of the file to write to
     * @param writeText what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
     * @param isAppending true: will append to the file if it exists, false: will create a new file
     * @param includeTimeStamp will include the timeStamp for when the method is called
     */
    public static void writeAFile(String fileName, String writeText, boolean isAppending, boolean includeTimeStamp ){

        // "\n" = System.lineSeparator()

        String time = "";
        if( includeTimeStamp ) {
            SimpleDateFormat dateFormatter = new SimpleDateFormat("MM-dd HH:mm:ss");
            Date date = new Date();
            time = dateFormatter.format(date) + " :: ";
        }

        //".../Internal Storage/";
        String path = Environment.getExternalStorageDirectory().getPath() + "/" + "FIRST" + "/";

        try {
            FileWriter writer = new FileWriter( new File( path + fileName ), isAppending );
            writer.write( time + writeText + System.lineSeparator() );
            writer.close();
        } catch ( IOException e ) {
            e.printStackTrace();
            Robot.writeToDefaultFile( e.getStackTrace().toString(), true, true);
        }

    }


    public void sleep(long millis){
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() < startTime + millis);
    }

    /**
     * opMode version of LinearOpmode's opModeIsActive
     * @return
     */
    public boolean opModeIsActive()
    {
        try {
            return ((LinearOpMode) opMode).opModeIsActive();
        } catch (ClassCastException e){
            return true;
        }
    }

    /**
     *
     * @param delay - delay/wait time in SECONDS
     */
    public void sleepRobot(long delay)
    {
        long setTime = System.currentTimeMillis();
        previousTime = opMode.getRuntime();

        while( (System.currentTimeMillis() - setTime)*1000 < (delay) && opModeIsActive())
            previousTime = opMode.getRuntime();

        telemetry.addData("Finished Sleep", "");
        telemetry.update();
    }

    public void sleepRobot2(long delay) {
        double currTime = opMode.getRuntime();
        double waitUntil = currTime + (double)(delay/1000);
        while( opMode.getRuntime() < waitUntil ) {}
    }



}
