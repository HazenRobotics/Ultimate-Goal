package org.firstinspires.ftc.teamcode.utils;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Robot;

import java.util.ArrayList;

public class TensorFlowUtil {

    private final String TENSOR_FLOW_MODEL_NAME = "UltimateGoal.tflite";

    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private Stack[] stackRecognitions;
    private ArrayList<Stack> stackRecognitions2 = new ArrayList<Stack>();

    private Vuforia vuforia = Vuforia.getInstance();

    public double loopRunTime = 0;

    // # of rings in starting stack
    Stack stack;

    TensorFlow tensorFlow;

    int tempLoop = 0;
    OpMode opMode;

    public enum Stack {
        NONE,
        SINGLE,
        QUAD
    }

    public TensorFlowUtil( HardwareMap hw, OpMode op ) {
        opMode = op;
    }

    public void initTensorFlow( HardwareMap hw ) {

        final String VUFORIA_KEY = hw.appContext.getResources().getString(R.string.vuforiakey);
        vuforia.setParameters(VUFORIA_KEY, "webcam", true, hw);
        vuforia.start();

        tensorFlow = new TensorFlow(TENSOR_FLOW_MODEL_NAME, 0.8f, true, hw, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

    }

    void startTF() {
        tensorFlow.activate();
    }

    Stack identifyObjects() {
        Recognition recognition = tensorFlow.getRecognition();
        if(recognition != null) {
            switch (recognition.getLabel()) {
                case "Single":
                    return Stack.SINGLE;
                case "Quad":
                    return Stack.QUAD;
            }
        }
        return Stack.NONE;
    }

    void objectDeterminationLoop(int loop ) { Log.e( "|-|-|-| ", "objectDeterminationLoop" );

        stackRecognitions = new Stack[loop];
        int singles = 0, quads = 0;

        double startTime = opMode.getRuntime();

        for( int i = 0; i < loop; i++ ) {
            if( stackRecognitions[i] != Stack.NONE) {
                stackRecognitions[i] = identifyObjects();
                if( stackRecognitions[i] == Stack.SINGLE)
                    singles++;
                else if( stackRecognitions[i] == Stack.QUAD)
                    quads++;
            }

            opMode.telemetry.addLine( "stackRecognition #" + tempLoop + " : " + stackRecognitions[tempLoop] );
            opMode.telemetry.update();
            Log.e( "|-|-|-| ", "stackRecognition #" + tempLoop + " : " + stackRecognitions[tempLoop++] );

            if( singles + quads >= 5 )
                break;
        }

        if( singles > quads ) {
            setStack(Stack.SINGLE);
            Robot.writeToDefaultFile( stack + "stack found", true, true );
        } else if( quads > singles) {
            setStack(Stack.QUAD);
            Robot.writeToDefaultFile( stack + "stack found", true, true );
        } else {
            setStack(Stack.NONE);
            Robot.writeToDefaultFile( stack + "stack found", true, true );
        }

        loopRunTime = opMode.getRuntime() - startTime;

        Log.e( "|-|-|-| ", "finished objectDeterminationLoop [in " + loopRunTime + "  seconds]" );
    }

    void objectDeterminationLoop() { Log.e( "|-|-|-| ", "objectDeterminationLoop2" );

        stackRecognitions2.add(identifyObjects() );

        while( stackRecognitions2.get(tempLoop) == Stack.NONE ) {
            opMode.telemetry.addLine( "stackRecognition #" + tempLoop + " : " + stackRecognitions2.get(tempLoop));
            opMode.telemetry.update();
            Log.e( "|-|-|-| ", "stackRecognition #" + tempLoop + " : " + stackRecognitions2.get(tempLoop++) );
            stackRecognitions2.add(identifyObjects() );
            //Log.e( "|-|-|-| ", "stackRecognition #" + i + " : " + ( stackRecognitions[i] != Stack.NONE ? "" + stackRecognitions[i] : "") );
        }
        opMode.telemetry.addLine( "stackRecognition #" + tempLoop + " : " + stackRecognitions2.get(tempLoop));
        opMode.telemetry.update();
        Log.e( "|-|-|-| ", "stackRecognition #" + tempLoop + " : " + stackRecognitions2.get(tempLoop++) );
        stackRecognitions2.add(identifyObjects() );

        int nones = 0, singles = 0, quads = 0;

        for( int i = 0; i < tempLoop; i++ ) {
            switch( stackRecognitions2.get(i) ) {
                /*case NONE:
                    nones++;
                    break;*/
                case SINGLE:
                    singles++;
                    break;
                case QUAD:
                    quads++;
                    break;
            }
        }

        /*
        String loggedArray = "";
        for( int i = 0; i < loop; i++ )
            loggedArray += ( "| Loop " + i + " :: " + stackRecognitions[i] + " |  -  " );
        Log.e( "|-|-|-| ", "here:" + loggedArray );
         */

        if( nones > singles && nones > quads )
            setStack( Stack.NONE );
        else if( singles > nones && singles > quads )
            setStack( Stack.SINGLE );
        else if( quads > singles && quads > nones )
            setStack( Stack.QUAD );
        else
            setStack( Stack.NONE ); //Log.e( "|-|-|-| ", "didn't set the stack to anything" );
        Log.e( "|-|-|-| ", "finished objectDeterminationLoop2" );


    }

    void stopTF() {
        tensorFlow.shutdown();
        if(vuforia.isRunning())
            vuforia.close();

    }

    /**
     *
     * @return the type of stack identified
     */
    public Stack getStack() {
        return this.stack;
    }

    public void setStack( Stack newStack ) {
        stack = newStack;
    }

    public void runStackDetection( int loops ) { Robot.writeToDefaultFile( "******** Main Class Started ********", true, true );

        //Log.e("|-|-|-| ", "runStackDetection();");

        startTF();

        objectDeterminationLoop(loops);

        Log.e( "|-|-|-| ", "finished objectDeterminationLoop [in " + loopRunTime + "  seconds]" );

        stopTF();
    }

}
