package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

/**
 * Stores information about the field so that it can represent a digital "map" of the field.
 * All measurements are in mm so that it is easier to use with vuforia
 */
public class FieldMap {
    //All measurements are in mm
    private static final float mmPerInch = (float)DistanceUnit.mmPerInch;
    //Field information
    private static final float fieldSize = 144 * mmPerInch;
    private static final float mmTargetHeight   = (6) * mmPerInch;
    private static final float halfField = fieldSize / 2;
    private static final float quadField  = fieldSize / 4;

    //Vuforia targets information
    public static class VuforiaTargets {
        public static final OpenGLMatrix BLUE_TOWER_GOAL_TARGET_POSITION = OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        public static final OpenGLMatrix RED_TOWER_GOAL_TARGET_POSITION = OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90));
        public static final OpenGLMatrix RED_ALLIANCE_TARGET_POSITION = OpenGLMatrix
                .translation(0, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180));
        public static final OpenGLMatrix BLUE_ALLIANCE_TARGET_POSITION = OpenGLMatrix
                .translation(0, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0));
        public static final OpenGLMatrix FRONT_WALL_TARGET_POSITION = OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90));

        public static final OpenGLMatrix[] TARGET_POSITIONS = {BLUE_TOWER_GOAL_TARGET_POSITION, RED_TOWER_GOAL_TARGET_POSITION,
                RED_ALLIANCE_TARGET_POSITION, BLUE_ALLIANCE_TARGET_POSITION, FRONT_WALL_TARGET_POSITION};
    }


    //Goals information
    public static class ScoringGoals {

        private static final float GOAL_LEFT_OFFSET = 37 * mmPerInch;
        private static final float LOW_GOAL_OFFSET = 17f * mmPerInch;
        private static final float MID_GOAL_OFFSET = 27f * mmPerInch;
        private static final float HIGH_GOAL_OFFSET = 33.5f * mmPerInch;

        public static final OpenGLMatrix BLUE_LOW_GOAL = OpenGLMatrix
                .translation(halfField, GOAL_LEFT_OFFSET, LOW_GOAL_OFFSET);
        public static final OpenGLMatrix BLUE_MID_GOAL = OpenGLMatrix
                .translation(halfField, -GOAL_LEFT_OFFSET, MID_GOAL_OFFSET);
        public static final OpenGLMatrix BLUE_HIGH_GOAL = OpenGLMatrix
                .translation(halfField, GOAL_LEFT_OFFSET, HIGH_GOAL_OFFSET);

        public static final OpenGLMatrix RED_LOW_GOAL = OpenGLMatrix
                .translation(halfField, -GOAL_LEFT_OFFSET, LOW_GOAL_OFFSET);
        public static final OpenGLMatrix RED_MID_GOAL = OpenGLMatrix
                .translation(halfField, GOAL_LEFT_OFFSET, MID_GOAL_OFFSET);
        public static final OpenGLMatrix RED_HIGH_GOAL = OpenGLMatrix
                .translation(halfField, -GOAL_LEFT_OFFSET, HIGH_GOAL_OFFSET);


        //Power Shot information
        private static final float CENTERLINE_POWERSHOT_OFFSET = 3.5f * mmPerInch;
        private static final float MIDDLE_POWERSHOT_OFFSET = 11f * mmPerInch;
        private static final float FAR_POWERSHOT_OFFSET = 18.5f * mmPerInch;
        private static final float POWERSHOT_HEIGHT_OFFSET = 23.5f * mmPerInch;

        public static final OpenGLMatrix BLUE_RIGHT_POWERSHOT = OpenGLMatrix
                .translation(halfField, CENTERLINE_POWERSHOT_OFFSET, POWERSHOT_HEIGHT_OFFSET);
        public static final OpenGLMatrix BLUE_MIDDLE_POWERSHOT = OpenGLMatrix
                .translation(halfField, MIDDLE_POWERSHOT_OFFSET, POWERSHOT_HEIGHT_OFFSET);
        public static final OpenGLMatrix BLUE_LEFT_POWERSHOT = OpenGLMatrix
                .translation(halfField, FAR_POWERSHOT_OFFSET, POWERSHOT_HEIGHT_OFFSET);

        public static final OpenGLMatrix RED_LEFT_POWERSHOT = OpenGLMatrix
                .translation(halfField, -CENTERLINE_POWERSHOT_OFFSET, POWERSHOT_HEIGHT_OFFSET);
        public static final OpenGLMatrix RED_MIDDLE_POWERSHOT = OpenGLMatrix
                .translation(halfField, -MIDDLE_POWERSHOT_OFFSET, POWERSHOT_HEIGHT_OFFSET);
        public static final OpenGLMatrix RED_RIGHT_POWERSHOT = OpenGLMatrix
                .translation(halfField, -FAR_POWERSHOT_OFFSET, POWERSHOT_HEIGHT_OFFSET);

    }

    //target zones
    public static class TargetGoalZones {

        private static final float EDGE_TARGETS_Y_OFFSET = 12f * mmPerInch;
        private static final float MIDDLE_TARGET_Y_OFFSET = 36f * mmPerInch;
        private static final float CENTERLINE_TARGET_X_OFFSET = 12f * mmPerInch;
        private static final float MIDDLE_TARGET_X_OFFSET = quadField;
        private static final float FAR_TARGET_X_OFFSET = 60f * mmPerInch;

        public static final OpenGLMatrix BLUE_CENTERLINE_GOAL_ZONE = OpenGLMatrix
                .translation(CENTERLINE_TARGET_X_OFFSET, EDGE_TARGETS_Y_OFFSET, 0);
        public static final OpenGLMatrix BLUE_MIDDLE_GOAL_ZONE = OpenGLMatrix
                .translation(MIDDLE_TARGET_X_OFFSET, MIDDLE_TARGET_Y_OFFSET, 0);
        public static final OpenGLMatrix BLUE_FAR_GOAL_ZONE = OpenGLMatrix
                .translation(FAR_TARGET_X_OFFSET, EDGE_TARGETS_Y_OFFSET, 0);

        public static final OpenGLMatrix RED_CENTERLINE_GOAL_ZONE = OpenGLMatrix
                .translation(CENTERLINE_TARGET_X_OFFSET, -EDGE_TARGETS_Y_OFFSET, 0);
        public static final OpenGLMatrix RED_MIDDLE_GOAL_ZONE = OpenGLMatrix
                .translation(MIDDLE_TARGET_X_OFFSET, -MIDDLE_TARGET_Y_OFFSET, 0);
        public static final OpenGLMatrix RED_FAR_GOAL_ZONE = OpenGLMatrix
                .translation(FAR_TARGET_X_OFFSET, -EDGE_TARGETS_Y_OFFSET, 0);

    }



    //Robot information
    public static class RobotInfo {
        //camera position in relation to the center of the robot
        private static final float CAMERA_FORWARD_DISPLACEMENT = 0f * mmPerInch, CAMERA_LEFT_DISPLACEMENT = 0f * mmPerInch, CAMERA_VERTICAL_DISPLACEMENT = 0f * mmPerInch;
        private static final float CAMERA_Y_ROTATE = -90f, CAMERA_Z_ROTATE = 0f, CAMERA_X_ROTATE = 0f;
        public static final OpenGLMatrix CAMERA_FROM_ROBOT = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, CAMERA_Y_ROTATE, CAMERA_Z_ROTATE, CAMERA_X_ROTATE));
        //the point at which the rings leave the shooter in relation to the center of the robot
        private static final float RING_LAUNCH_FORWARD_DISPLACEMENT = 0f * mmPerInch, RING_LAUNCH_LEFT_DISPLACEMENT = 0f * mmPerInch, RING_LAUNCH_VERTICAL_DISPLACEMENT = 0f * mmPerInch;
        public static final OpenGLMatrix RING_LAUNCH_POSITION_FROM_ROBOT = OpenGLMatrix
                .translation(RING_LAUNCH_FORWARD_DISPLACEMENT, RING_LAUNCH_LEFT_DISPLACEMENT, RING_LAUNCH_VERTICAL_DISPLACEMENT);

        //angle of the shooter from the ground
        public static final float SHOOTER_ANGLE = 30;

        /**
         * The robots position. A null value means that the robot does not currently know where it is on the field
         */
        public static OpenGLMatrix robotLocation = null;

        private static OpenGLMatrix ringLaunchPosition = null;


        /**
         * Gets the point at which the disks leave the launcher relative to the field
         * @return position at which the disks leave the launcher
         */
        public static OpenGLMatrix getRingLaunchPointPosition(){
            ringLaunchPosition = (OpenGLMatrix) robotLocation.added(RING_LAUNCH_POSITION_FROM_ROBOT);
            return ringLaunchPosition;
        }
    }

    public static double getDistanceBetweenTwoPoints(VectorF point1, VectorF point2){
        double x = getGroundDistanceBetweenTwoPoints(point1, point2);
        double y = getHeightDifferenceBetweenTwoPoints(point1, point2);
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }

    public static double getGroundDistanceBetweenTwoPoints(VectorF point1, VectorF point2){
        return Math.sqrt(Math.pow(point2.get(0) - point1.get(0), 2) + Math.pow(point2.get(1) - point1.get(1), 2));
    }

    public static double getHeightDifferenceBetweenTwoPoints(VectorF point1, VectorF point2){
        return point2.get(2) - point1.get(2);
    }


}