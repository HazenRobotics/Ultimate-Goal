package org.firstinspires.ftc.teamcode;

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

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

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
    private final String VUFORIA_TRACKABLES_ASSET_NAME = "UltimateGoal";

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

        final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);

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
