package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.TensorFlow;
import org.firstinspires.ftc.teamcode.utils.VuforiaNavigation;

/**
 * This class sets up and manages a robot
 */
public class Robot {
    HardwareMap hardwareMap;

    //drive
    Drive drive;

    //mechanisms
    //RingShooter shooter;
    //GoalLift lift;

    //Vuforia
    VuforiaNavigation vuforiaNavigation;
    private final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);
    private final String VUFORIA_TRACKABLES_ASSET_NAME = "Ultimate Goal";

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

        //drive type
        drive = new MecanumDrive(hw);

        //mechanisms
        //shooter = new RingShooter(hw);
        //lift = new GoalLift(hw);

        //Vuforia.startVuforiaEngine(VUFORIA_KEY, "webcam", true, hw);
        //vuforiaNavigation = new VuforiaNavigation(VUFORIA_TRACKABLES_ASSET_NAME);
        //tfod = new TensorFlow(TFOD_MODEL_ASSET_NAME, 0.8f, true, hardwareMap, TFOD_MODEL_LABELS);
    }

}
