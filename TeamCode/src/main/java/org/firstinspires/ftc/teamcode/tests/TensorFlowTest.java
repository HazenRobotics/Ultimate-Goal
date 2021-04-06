package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.TensorFlow;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

@TeleOp(name="TensorFlow Test")
@Disabled
public class TensorFlowTest extends OpMode {

    private final String TENSOR_FLOW_MODEL_NAME = "UltimateGoal";

    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private Vuforia vuforia = Vuforia.getInstance();

    TensorFlow tensorFlow;
    @Override
    public void init() {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);
        vuforia.setParameters(VUFORIA_KEY, "webcam", true, hardwareMap);
        vuforia.start();

        tensorFlow = new TensorFlow(TENSOR_FLOW_MODEL_NAME, 0.8f, true, hardwareMap, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    
        Robot.writeToMatchDefaultFile( "Init Finished", true );
    }

    @Override
    public void start() {
        tensorFlow.activate();
    }


    @Override
    public void loop() {
        if(tensorFlow.getRecognition() != null) {
            Recognition recognition = tensorFlow.getRecognition();
            telemetry.addData(String.format("label (%d)"), recognition.getLabel());
            telemetry.addData(String.format("  left,top (%d)"), "%.03f , %.03f",
                    recognition.getLeft(), recognition.getTop());
            telemetry.addData(String.format("  right,bottom (%d)"), "%.03f , %.03f",
                    recognition.getRight(), recognition.getBottom());
        }
        else {
            telemetry.addData("No Recognitions", null);
        }
    }

    @Override
    public void stop() {
        tensorFlow.shutdown();
        if(vuforia.isRunning()) {
            vuforia.close();
        }
    }

}
