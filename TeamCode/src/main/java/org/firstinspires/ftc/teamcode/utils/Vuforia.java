package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * A class that holds an instance of the vuforia engine.
 * {@link #startVuforiaEngine(String, String, boolean, HardwareMap)}  Vuforia.startVuforiaEngine()} method must be called before instantiating {@link VuforiaLocalization} or {@link TensorFlow}
 */
public class Vuforia {

    public static VuforiaLocalizerPlus vuforia = null;

    private static VuforiaLocalizer.Parameters parameters;

    /**
     * Starts the vuforia engine with the given parameters
     * @param vuforiaKey vuforia liscense key
     * @param webcamName name of the webcam in the robot configuration
     * @param monitorCamera if the camera monitor should run
     * @param hw robot's hardware map
     */
    public static void startVuforiaEngine(String vuforiaKey, String webcamName, boolean monitorCamera, HardwareMap hw){

        parameters = monitorCamera == true? new VuforiaLocalizer.Parameters(hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName())) : new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = vuforiaKey;
        parameters.cameraName = hw.get(WebcamName.class, webcamName);
        parameters.useExtendedTracking = false;

        vuforia = new VuforiaLocalizerPlus(parameters);
    }

    public static boolean isRunning(){
        return vuforia != null;
    }

    public static void stopVuforiaEngine(){
        vuforia.close();
        vuforia = null;
    }
}
