package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.Vuforia;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalization;

@TeleOp(name="Vuforia Test")
@Disabled
public class VuforiaTest extends OpMode {

    private final String VUFORIA_TRACKABLES_ASSET_NAME = "UltimateGoal";

    VuforiaLocalization vuforiaLocalization;
    Vuforia vuforia = Vuforia.getInstance();


    @Override
    public void init() {

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        final String VUFORIA_KEY = hardwareMap.appContext.getResources().getString(R.string.vuforiakey);
        vuforia.setParameters(VUFORIA_KEY, "webcam", true, hardwareMap);
        vuforia.start();

        vuforiaLocalization = new VuforiaLocalization(VUFORIA_TRACKABLES_ASSET_NAME);
    
        Robot.writeToMatchFile( "Init Finished", true );
    }
    @Override
    public void start(){
        vuforiaLocalization.activateTracking();
    }

    @Override
    public void loop() {
        if(vuforiaLocalization.getVisibleTarget() != null) {
            telemetry.addData("Visible Vuforia Mark", vuforiaLocalization.getVisibleTarget().getName());
            vuforiaLocalization.updateRobotLocation();
            telemetry.addData("Robot Position", FieldMap.toPose2d(vuforiaLocalization.getRobotPosition(), vuforiaLocalization.getRobotRotation()));
        }
        else{
            telemetry.addData("Visible Vuforia Mark", "none");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        vuforiaLocalization.deactivateTracking();
        if(vuforia.isRunning()) {
            vuforia.close();
        }
    }
}
