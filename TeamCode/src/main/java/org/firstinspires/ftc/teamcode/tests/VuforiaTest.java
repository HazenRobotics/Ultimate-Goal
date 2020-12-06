package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.utils.Vuforia;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalization;

@TeleOp(name="Vuforia Test")
public class VuforiaTest extends OpMode {

    private final String VUFORIA_TRACKABLES_ASSET_NAME = "UltimateGoal";

    VuforiaLocalization vuforiaLocalization;


    @Override
    public void init() {
        final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);
        Vuforia.startVuforiaEngine(VUFORIA_KEY, "webcam", true, hardwareMap);

        vuforiaLocalization = new VuforiaLocalization(VUFORIA_TRACKABLES_ASSET_NAME);
    }
    @Override
    public void start(){
        vuforiaLocalization.activateTracking();
    }

    @Override
    public void loop() {
        if(vuforiaLocalization.getVisibleTarget() != null) {
            telemetry.addData("Visible Vuforia Mark", vuforiaLocalization.getVisibleTarget().getName());
        }
        else{
            telemetry.addData("Visible Vuforia Mark", "none");
        }
        telemetry.update();
    }

    @Override
    public void stop(){
        vuforiaLocalization.deactivateTracking();
        if(Vuforia.isRunning()){
            Vuforia.stopVuforiaEngine();
        }
    }
}
