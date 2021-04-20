package org.firstinspires.ftc.teamcode.robots;

import android.os.Environment;
import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.LogUtil;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;

/**
 * This class sets up and manages a robot
 */
public abstract class Robot {

    HardwareMap hardwareMap;
    OpMode opMode;
    Telemetry telemetry;

    // drive
    public Drive driveTrain;

    public static final TensorFlowUtil.Stack STACK_NONE = TensorFlowUtil.Stack.NONE;
    public static final TensorFlowUtil.Stack STACK_SINGLE = TensorFlowUtil.Stack.SINGLE;
    public static final TensorFlowUtil.Stack STACK_QUAD = TensorFlowUtil.Stack.QUAD;

    public static final RingShooter.PusherPosition PUSHER_PUSHED = RingShooter.PusherPosition.PUSHED;
    public static final RingShooter.PusherPosition PUSHER_RETRACTED = RingShooter.PusherPosition.RETRACTED;

    //public static final RingBlocker.BlockerPosition BLOCKER_BLOCKED = RingBlocker.BlockerPosition.BLOCKED;
    //public static final RingBlocker.BlockerPosition BLOCKER_RETRACTED = RingBlocker.BlockerPosition.RETRACTED;

    public static final GoalLift.ClawPosition CLAW_OPEN = GoalLift.ClawPosition.OPEN;
    public static final GoalLift.ClawPosition CLAW_CLOSED = GoalLift.ClawPosition.CLOSED;

    public static final GoalLift.LiftPosition LIFT_LIFTED = GoalLift.LiftPosition.LIFTED;
    public static final GoalLift.LiftPosition LIFT_LOWERED = GoalLift.LiftPosition.LOWERED;

    /**
     * Creates a Robot
     *
     * @param hw robot's hardware map
     */
    public Robot( HardwareMap hw, OpMode op ) {
        this.hardwareMap = hw;
        this.opMode = op;
        telemetry = opMode.telemetry;

        //vuforiaKey = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);

        //Bulk Caching to decrease cycle times
        for( LynxModule module : hw.getAll( LynxModule.class ) ) {
            module.setBulkCachingMode( LynxModule.BulkCachingMode.AUTO );
        }
    }

    public static void createMatchLogFile( String className ) {
        LogUtil.createMatchLogFile( className );
    }

    /**
     * writes to the default match file
     *
     * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
     * @param includeTimeStamp will include the timeStamp for when the method is called
     */
    public static void writeToMatchFile( String writeText, boolean includeTimeStamp ) {
        LogUtil.writeToMatchFile( writeText, includeTimeStamp );
    }

    /**
     * writes to the default file *robotLog.txt
     *
     * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
     * @param isAppending      true: will append to the file if it exists, false: will create a new file
     * @param includeTimeStamp will include the timeStamp for when the method is called
     */
    public static void writeToDefaultFile( String writeText, boolean isAppending, boolean includeTimeStamp ) {
        LogUtil.writeToDefaultFile( writeText, isAppending, includeTimeStamp );
    }

    /**
     * @param fileName         the name of the file to write to
     * @param writeText        what the method will write to the fill (plus the timeStamp if includeTimeStamp is true)
     * @param isAppending      true: will append to the file if it exists, false: will create a new file
     * @param includeTimeStamp will include the timeStamp for when the method is called
     */
    public static void writeAFile( String fileName, String writeText, boolean isAppending, boolean includeTimeStamp ) {
        LogUtil.writeAFile( fileName, writeText, isAppending, includeTimeStamp );
    }

    public void sleep( long millis ) {
        long startTime = System.currentTimeMillis();
        while( System.currentTimeMillis() < startTime + millis && opModeIsActive() ) ;
    }

    /**
     * opMode version of LinearOpmode's opModeIsActive
     *
     * @return
     */
    public boolean opModeIsActive() {
        try {
            return ((LinearOpMode) opMode).opModeIsActive();
        } catch( ClassCastException e ) {
            return true;
        }
    }

    /**
     * @param delay - delay/wait time in SECONDS
     */
    public void sleepRobot( long delay ) {
        long setTime = System.currentTimeMillis();
        while( (System.currentTimeMillis() - setTime) * 1000 < (delay) && opModeIsActive() ) ;
    }

    public void sleepRobot2( long delay ) {
        double curTime = opMode.getRuntime();
        double waitUntil = curTime + (double) (delay / 1000);
        while( opMode.getRuntime() < waitUntil ) ;
    }


}
