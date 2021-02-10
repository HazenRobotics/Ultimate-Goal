package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;

public class TensorFlowUtil {

    private final String TENSOR_FLOW_MODEL_NAME = "UltimateGoal.tflite";

    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private Vuforia vuforia = Vuforia.getInstance();

    // # of rings in starting stack
    Stack stack;

    TensorFlow tensorFlow;

    public enum Stack {
        NONE,
        SINGLE,
        QUAD
    }


    public TensorFlowUtil( HardwareMap hw ) {

        initTensorFlow(hw);

    }


    public void initTensorFlow( HardwareMap hw ) {

        final String VUFORIA_KEY = hw.appContext.getResources().getString(R.string.vuforiakey);
        vuforia.setParameters(VUFORIA_KEY, "webcam", true, hw);
        vuforia.start();

        tensorFlow = new TensorFlow(TENSOR_FLOW_MODEL_NAME, 0.8f, true, hw, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }


    void startTFOD() {
        tensorFlow.activate();
    }


    void identifyObjects() {
        if(tensorFlow.getRecognition() != null) {
            Recognition recognition = tensorFlow.getRecognition();
            setStack( recognition );
        }
        else {
            stack = Stack.NONE;
        }
    }

    void stopTFOD() {
        tensorFlow.shutdown();
        if(vuforia.isRunning()) {
            vuforia.close();
        }
    }


    /**
     *
     * @return the type of stack identified
     */
    public Stack getStack() {
        return this.stack;
    }

    public void setStack( Recognition tfodResult ) {
        switch (tfodResult.getLabel()) {
            case "Single":
                stack = Stack.SINGLE;
                break;
            case "Quad":
                stack = Stack.QUAD;
                break;
        }
    }



    public static void runStackDetection( int loopsIfNotFound ){


        //startTFOD();


        //identifyObjects();

        //stopTFOD();

    }

}
