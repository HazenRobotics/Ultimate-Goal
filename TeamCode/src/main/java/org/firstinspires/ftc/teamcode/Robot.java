package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.TensorFlow;
import org.firstinspires.ftc.teamcode.utils.Vuforia;
import org.firstinspires.ftc.teamcode.utils.VuforiaNavigation;

import java.util.function.Function;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

/**
 * This class sets up and manages a robot
 */
public abstract class Robot {
    HardwareMap hardwareMap;
    OpMode opMode;
    Telemetry telemetry;

    //time
    double previousTime;

    //drive
    Drive driveTrain;

    // gyro
    BNO055IMU gyro;

    //

    //mechanisms
    //RingShooter shooter;
    //GoalLift lift;

    //Vuforia
    VuforiaNavigation vuforiaNavigation;
    private final String VUFORIA_TRACKABLES_ASSET_NAME = "Ultimate Goal";
    String vuforiaKey;

    //Tensor Flow
    TensorFlow tfod;
    private final String TFOD_MODEL_ASSET_NAME = "UltimateGoal.tflite";
    private final String[] TFOD_MODEL_LABELS = {"Quad", "Single"};


    /**
     * Creates a Robot
     * @param hw robot's hardware map
     */
    public Robot( HardwareMap hw, OpMode op ) {
        this.hardwareMap = hw;
        this.opMode = op;
        telemetry = opMode.telemetry;

        gyro = hardwareMap.get( BNO055IMU.class, "imu" );

        vuforiaKey = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);

        //drive type

        //mechanisms
        //shooter = new RingShooter(hw);
        //lift = new GoalLift(hw);

        //Vuforia.startVuforiaEngine(VUFORIA_KEY, "webcam", true, hw);
        //vuforiaNavigation = new VuforiaNavigation(VUFORIA_TRACKABLES_ASSET_NAME);
        //tfod = new TensorFlow(TFOD_MODEL_ASSET_NAME, 0.8f, true, hardwareMap, TFOD_MODEL_LABELS);


        //Bulk Caching to decrease cycle times
        for (LynxModule module : hw.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
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

        while(System.currentTimeMillis() - setTime < (delay) && opModeIsActive())
            previousTime = opMode.getRuntime();

        telemetry.addData("Finished Sleep", "");
        telemetry.update();
    }



}
