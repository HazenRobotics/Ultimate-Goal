package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * A class for tracking Vuforia targets on the field
 */
public class VuforiaNavigation {

    VuforiaLocalizer vuforia;

    VuforiaTrackables targets;

    public List<VuforiaTrackable> trackables;

    VuforiaLocalizer.Parameters parameters;


    /**
     * Creates a VuforiaTargetTracker
     * @param vuforiaKey vuforia liscense key
     * @param webcamName name of the webcam in the robot configuration
     * @param cameraMonitorViewId id of the camera monitor, null if no camera monitoring is wanted
     * @param trackablesAssetName name of the tensorflow asset file
     * @param robotFromCamera a transformation matrix describing where the phone is on the robot
     */
    public VuforiaNavigation(String vuforiaKey, String webcamName, @Nullable Integer cameraMonitorViewId, String trackablesAssetName,  OpenGLMatrix robotFromCamera, HardwareMap hw){
        startVuforiaEngine(vuforiaKey, webcamName, cameraMonitorViewId, hw);

        loadTrackables(trackablesAssetName);

        setCameraPosition(robotFromCamera);
    }

    /**
     * Starts the vuforia engine with the given parameters
     * @param vuforiaKey vuforia liscense key
     * @param webcamName name of the webcam in the robot configuration
     * @param cameraMonitorViewId id of the camera monitor, null if no camera monitoring is wanted
     */
    private void startVuforiaEngine(String vuforiaKey, String webcamName, @Nullable Integer cameraMonitorViewId, HardwareMap hw){

        parameters = cameraMonitorViewId != null? new VuforiaLocalizer.Parameters(cameraMonitorViewId) : new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = vuforiaKey;
        parameters.cameraName = hw.get(WebcamName.class, webcamName);
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Creates the trackable objects from a tensorflow asset
     * @param assetName name of the tensorflow asset file
     */
    private void loadTrackables(String assetName){

        targets = vuforia.loadTrackablesFromAsset(assetName);

        trackables = new ArrayList<VuforiaTrackable>();

        for(int i = 0; i < targets.size(); i++){
            trackables.add(targets.get(i));
        }
    }

    /**
     * Tells tensorflow where the camera is located on the robot
     * @param robotFromCamera a transformation matrix describing where the phone is on the robot
     */
    private void setCameraPosition(OpenGLMatrix robotFromCamera){
        for(VuforiaTrackable trackable : trackables){
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }
    }

    /**
     * Sets the trackables to the new set inputted
     * @param newTrackables modified list of trackables
     */
    public void setTrackables(List<VuforiaTrackable> newTrackables){
        trackables = newTrackables;
    }

    /**
     * Gets the list of trackables
     * @return list of trackables
     */
    public List<VuforiaTrackable> getTrackables(){
        return trackables;
    }

    /**
     * Activates tracking of the targets
     */
    public void activateTracking(){
        targets.activate();
    }

    /**
     * Deactivates tracking of the targets
     */
    public void deactivateTracking(){
        targets.deactivate();
    }

    /**
     * Checks if a Vuforia target is currently visible and returns the one that is
     * @return the trackable that is visible. If no trackable is visible, returns null
     */
    public VuforiaTrackable getVisibleTarget(){

        for(VuforiaTrackable trackable : trackables){
            if(((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()){
                return trackable;
            }
        }
        return null;
    }

    /**
     * Gets a transformation matrix of the robots location data
     * @return robot's current location transform
     */
    private OpenGLMatrix getRobotLocationTransform(){
        VuforiaTrackable currentVisibleTarget = getVisibleTarget();
        if(currentVisibleTarget == null){
            return null;
        }
        return ((VuforiaTrackableDefaultListener)currentVisibleTarget.getListener()).getUpdatedRobotLocation();
    }

    /**
     * Gets the robot's position, expressed as a {@link VectorF}
     * @return robot's current position
     */
    public VectorF getRobotPosition(){
        OpenGLMatrix robotLocation = getRobotLocationTransform();
        if(robotLocation == null){
            return null;
        }
        return robotLocation.getTranslation();
    }

    /**
     * Gets the robot's rotation, expressed as a {@link Orientation}
     * @return robot's current orientation
     */
    public Orientation getRobotRotation(){
        OpenGLMatrix robotLocation = getRobotLocationTransform();
        if(robotLocation == null){
            return null;
        }
        return Orientation.getOrientation(robotLocation, EXTRINSIC, XYZ, DEGREES);
    }
}
