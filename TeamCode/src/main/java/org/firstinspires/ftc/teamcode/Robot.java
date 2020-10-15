package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drives.Drive;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.utils.VuforiaNavigation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

/**
 * This class sets up and manages a robot
 */
public class Robot {
    HardwareMap hardwareMap;

    //drive
    Drive drive;

    //mechanisms
    //RingShooter shooter;
    //GoalLift lift;

    //Vuforia
    VuforiaNavigation vuforiaNavigation;
    private final String VUFORIA_KEY = "ASZZxs//////AAABmZH+VLl/YE2ekt+27Dayd/8VPb+94QyyfpWCvrKJmZO4LmYykX4/iMfk2R7qvaDVllKYEjyuF7pPxkH4TMmXulQMt+/snHA1TsS7+lWVL2naborYEbp2YjAzjbyqLS0bB+KpVH4o0piCviop3XuHFE+b+1L0wyiHTVmwnEC5eyGuwVPB2iL2JrsXqYQcaKFD4EKr9hUarDSc4tNSTrwknvfCHsGWfTvI+euQAUxKzBOCKyApx4x1Y3xLZyj1J6Mk2/fTsfHTuBUoGfHzdinX7eOKo8CJ/uo66pvxBpU6fH/IeQBR6JuqvUNiSKP5lUI+c8bHR9oTd9hFxboX41DzpDJAmex0SNiODJwKGMrCKC+d";
    //displacement of camera from the center of the robot in inches
    private static final float CAMERA_FORWARD_DISPLACEMENT = 0 * (float) mmPerInch; //displacement from center of robot
    private static final float CAMERA_LEFT_DISPLACEMENT = 0 * (float) mmPerInch; //displacement from center of robot
    private static final float CAMERA_VERTICAL_DISPLACEMENT = 0 * (float) mmPerInch; //displacement from ground
    private OpenGLMatrix cameraDisplacement = OpenGLMatrix.translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT).multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));
    //height of vuforia targets
    private static final float mmTargetHeight   = (6) * (float) mmPerInch;


    /**
     * Creates a Robot
     * @param hw robot's hardware map
     */
    public Robot( HardwareMap hw ) {
        this.hardwareMap = hw;

        //drive type
        drive = new MecanumDrive(hw);

        //mechanisms
        //shooter = new RingShooter(hw);
        //lift = new GoalLift(hw);

        vuforiaNavigation = new VuforiaNavigation(VUFORIA_KEY, "webcam", null, "UltimateGoal", cameraDisplacement, hardwareMap);
    }

}
