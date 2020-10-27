package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    //drive
    Drive drive;

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
    public Robot( HardwareMap hw ) {
        this.hardwareMap = hw;

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
}
