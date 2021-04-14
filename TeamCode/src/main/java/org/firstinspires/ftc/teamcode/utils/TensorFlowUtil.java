package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    
    private Vuforia vuforia = Vuforia.getInstance();
    
    // class specific
    private Stack[] stackRecognitions;
    private ArrayList<Stack> infiniteStackRecognitions;

    private int defaultLoops = 20000, totalLoops = 0;
    private double loopRunTime = 0;
    
    // Asynchronous Threads
    Thread dOLoopAsync = new Thread(() -> determineObjectLoop()); // technically do the same thing on init
    Thread dOLoopNumAsync = new Thread(() -> determineObjectLoop(defaultLoops)); // technically do the same thing on init but can be changed later
    Thread dOWhileLoopAsync = new Thread(() -> determineObjectWhileLoop());

    // # of rings in starting stack
    Stack stack;

    TensorFlow tensorFlow;
    OpMode opMode;
    HardwareMap hardwareMap;

    public enum Stack {
        NONE,
        SINGLE,
        QUAD
    }

    public TensorFlowUtil( HardwareMap hw, OpMode op ) {
        opMode = op;
        hardwareMap = hw;
    }

    public void initTensorFlow() {

        if(!vuforia.isRunning()) {
            final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);
            vuforia.setParameters(VUFORIA_KEY, "webcam", true, hardwareMap);
            vuforia.start();
        }

        tensorFlow = new TensorFlow(TENSOR_FLOW_MODEL_NAME, 0.8f, true, hardwareMap, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
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
    
    void determineObjectLoop() {
        determineObjectLoop( defaultLoops );
    }

    void determineObjectLoop( int loops ) { Robot.writeToMatchFile( "objectDeterminationLoop", true );

        stackRecognitions = new Stack[loops];
        int singles = 0, quads = 0;
        
        totalLoops = 0;
        double startTime = opMode.getRuntime();

        for( int i = 0; i < loops; i++ ) {
            stackRecognitions[i] = identifyObjects();
            if( stackRecognitions[i] == Stack.SINGLE)
                singles++;
            else if( stackRecognitions[i] == Stack.QUAD)
                quads++;

            opMode.telemetry.addLine( "stackRecognition #" + (totalLoops = i) + " : " + stackRecognitions[i] );
            opMode.telemetry.update();

            if( singles + quads >= 5 )
                break;
        }
    
        setStack(Stack.NONE);
        if( singles > quads )
            setStack(Stack.SINGLE);
        else if( quads > singles)
            setStack(Stack.QUAD);

        loopRunTime = opMode.getRuntime() - startTime;

        logAndPrint( stack + " stack found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
    }

    void determineObjectWhileLoop() { Robot.writeToMatchFile( "objectDeterminationWhileLoop", true );
    
        infiniteStackRecognitions = new ArrayList<Stack>();
        
        totalLoops = 0;
        double startTime = opMode.getRuntime();

        while( !((LinearOpMode)opMode).isStarted() || totalLoops < defaultLoops ) {
    
            infiniteStackRecognitions.add(identifyObjects() );
            if( totalLoops++ >= defaultLoops ) {
                infiniteStackRecognitions.remove( 0 );
                totalLoops--;
            }
        }

        int singles = 0, quads = 0;

        for(int i = 0; i < totalLoops; i++ ) {
            opMode.telemetry.addLine( "stackRecognition #" + i + " : " + infiniteStackRecognitions.get(i));
            opMode.telemetry.update();
            switch( infiniteStackRecognitions.get(i) ) {
                case SINGLE:
                    singles++;
                    break;
                case QUAD:
                    quads++;
                    break;
            }
        }
    
        setStack(Stack.NONE);
        if( singles > quads )
            setStack(Stack.SINGLE);
        else if( quads > singles)
            setStack(Stack.QUAD);
    
        loopRunTime = opMode.getRuntime() - startTime;
    
        logAndPrint( stack + " stack found [in " + totalLoops + " loops & " + loopRunTime + " seconds]", true );
    }
    
    /**
     * runs determineObjectLoop() asynchronously (on another thread)
     * @return whether any object detection loops are running
     */
    public boolean determineObjectLoopAsync() {
        boolean threadsActive = tensorFlowUtilThreadsActive();
        if( !threadsActive )
            dOLoopAsync.start();
        return threadsActive;
    }
    
    /**
     * runs determineObjectLoop() asynchronously (on another thread)
     * @param loops how many times to loop scanning the ring stack
     * @return whether any object detection loops are running
     */
    public boolean determineObjectLoopAsync( int loops ) {
        boolean threadsActive = tensorFlowUtilThreadsActive();
        if( !threadsActive )
            (dOLoopNumAsync = new Thread(() -> determineObjectLoop( loops ))).start();
        return threadsActive;
    }
    
    /**
     * runs determineObjectWhileLoop() asynchronously (on another thread)
     * @return whether any object detection loops are running
     */
    public boolean determineObjectWhileLoopAsync() {
        boolean threadsActive = tensorFlowUtilThreadsActive();
        if( !threadsActive )
            dOWhileLoopAsync.start();
        return threadsActive;
    }
    
    /**
     *
     * @return whether any object detection loops are running
     */
    public boolean tensorFlowUtilThreadsActive() { // returns if any of them are active
        return dOWhileLoopAsync.isAlive() || dOLoopAsync.isAlive() || !dOLoopNumAsync.isAlive();
    }

    public void waitForIdle() {

    }

    void stopTF() {
        tensorFlow.shutdown();
    }

    public void deactivateTensorFlow() {
        stopTF();
    }

    public void setStack( Stack newStack ) {
        this.stack = newStack;
    }
    
    public Stack getStack() {
        return this.stack;
    }

    public void setDefaultLoops(int newLoop  ) {
        defaultLoops = newLoop;
    }
    
    public int getDefaultLoops() {
        return defaultLoops;
    }
    
    public int getTotalLoops() {
        return totalLoops;
    }

    public void setZoom( double zoom, double aspectRatio ) {
        tensorFlow.setZoom( zoom, aspectRatio );
    }

    public void setZoom( double zoom ) {
        tensorFlow.setZoom( zoom, 16.0/9.0 );
    }

    public void runStackDetection( int loops ) { Robot.writeToMatchFile( "runStackDetection()", true );

        startTF();

        determineObjectLoop(loops);

        stopTF();
    }

    public void runWhileStackDetection() { Robot.writeToMatchFile( "runStackDetection()", true );
        
        startTF();

        //determineObjectWhileLoopAsync();

        determineObjectWhileLoop();
        
        stopTF();
    }
    
    public void logAndPrint( String text, boolean includeTimeStamp ) {
        
        Robot.writeToMatchFile( text, includeTimeStamp );
        opMode.telemetry.addLine(text);
        opMode.telemetry.update();
    }

}
