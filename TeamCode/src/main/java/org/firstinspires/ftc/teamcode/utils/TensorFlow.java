package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * A class for utilizing a Tensor Flow model.
 * {@link Vuforia#startVuforiaEngine(String, String, boolean, HardwareMap) Vuforia.startVuforiaEngine()} must be called before this class is instantiated
 */
public class TensorFlow {

    private TFObjectDetector tfod;

    private boolean isActive = false;

    private List<Recognition> recognitions = null;
    private List<Recognition> updatedRecognitions;

    /**
     * Creates a TensorFlow
     * @param tfodModelAssetName name of the model asset, found in the assets folder
     * @param minResultConfidence minimum result confidence to be considered a "Recognition". Value from 0 to 1.
     * @param monitorCamera if the camera monitor should run
     * @param hw robot's hardware map
     * @param labels labels of the entries in the .tflite file
     */
    public TensorFlow(String tfodModelAssetName, float minResultConfidence, boolean monitorCamera, HardwareMap hw, String... labels){

        initTfod(tfodModelAssetName, minResultConfidence, monitorCamera, hw, labels);


    }

    /**
     * Initializes Tensor Flow for use
     * @param tfodModelAssetName name of the model asset, found in the assets folder
     * @param minResultConfidence minimum result confidence to be considered a "Recognition". Value from 0 to 1.
     * @param monitorCamera if the camera monitor should run
     * @param hw robot's hardware map
     * @param labels labels of the entries in the .tflite file
     */
    private void initTfod(String tfodModelAssetName, float minResultConfidence, boolean monitorCamera, HardwareMap hw, String... labels){
        TFObjectDetector.Parameters tfodParameters = monitorCamera == true ? new TFObjectDetector.Parameters(hw.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hw.appContext.getPackageName())) : new TFObjectDetector.Parameters();
        tfodParameters.minResultConfidence = minResultConfidence;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, Vuforia.vuforia);
        tfod.loadModelFromAsset(tfodModelAssetName, labels);
    }

    /**
     * Activates object recognition
     */
    public void activate(){
        tfod.activate();
    }

    /**
     * Stops object recognition
     */
    public void shutdown(){
        tfod.shutdown();
    }

    /**
     * Tells if object recognition is currently running
     * @return whether Tensor Flow is active or not
     */
    public boolean isActive(){
        return isActive;
    }

    /**
     * Gets the current recognition
     * @return current recognition
     */
    public Recognition getRecognition(){
        updateRecognitions();
        if(recognitions == null){
            return  null;
        }
        if(recognitions.size() > 1){
            Recognition mostConfidentRecognition = null;
            for(Recognition recognition : recognitions){
                if(mostConfidentRecognition == null || recognition.getConfidence() > mostConfidentRecognition.getConfidence()){
                    mostConfidentRecognition = recognition;
                }
            }
            return  mostConfidentRecognition;
        }
        return recognitions.get(0);
    }

    /**
     * Updates the current recognition
     */
    public void updateRecognitions(){
        recognitions = tfod.getRecognitions();
    }
}
