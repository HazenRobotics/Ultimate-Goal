package org.firstinspires.ftc.teamcode.tests;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.TensorFlow;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

@TeleOp(name="TensorFlow Test")
//@Disabled
public class TensorFlowTest extends OpMode {

    private static final String TENSOR_FLOW_MODEL_NAME = "UltimateGoal.tflite";

    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private Vuforia vuforia = Vuforia.getInstance();
    
    GamepadEvents gamepad1;

    TensorFlow tensorFlow;
    
    private double zoom = 2;
    
    @Override
    public void init() {

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        gamepad1 = new GamepadEvents(super.gamepad1);

        final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);
        vuforia.setParameters(VUFORIA_KEY, "webcam", true, hardwareMap);
        vuforia.start();

        tensorFlow = new TensorFlow(TENSOR_FLOW_MODEL_NAME, 0.8f, true, hardwareMap, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    
        Robot.writeToMatchFile( "Init Finished", true );

        telemetry.addLine( "Init finished" );
        telemetry.update();
    }

    @Override
    public void start() {
        tensorFlow.activate();
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {

        telemetry.addLine( "dpad up :: + 0.5" );
        telemetry.addLine( "dpad down :: - 0.5" );
        telemetry.addLine( "------------------" );
        telemetry.addLine( "Zoom :: " + zoom );

        if(tensorFlow.getRecognition() != null) {
            Recognition recognition = tensorFlow.getRecognition();
            telemetry.addData( String.format("label (%d)", 0), recognition.getLabel() );
            telemetry.addData( String.format("  left,top (%d)", 0), "%.03f , %.03f", recognition.getLeft(), recognition.getTop() );
            telemetry.addData( String.format("  right,bottom (%d)", 0), "%.03f , %.03f", recognition.getRight(), recognition.getBottom() );
        } else
            telemetry.addData("No Recognitions", null);

        if( gamepad1.dpad_up.onPress())
            zoom += 0.5;
        else if( gamepad1.dpad_down.onPress() && zoom >= 1)
            zoom -= 0.5;

        tensorFlow.setZoom(zoom, 16.0/9.0);
        gamepad1.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        tensorFlow.shutdown();
        if(vuforia.isRunning())
            vuforia.close();
    }

}
