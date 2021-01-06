package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import java.lang.Math;

public class ShootingMath {

    static final double g = -9.81; // gravity in m/s^2


    /**
     * Calculates the angle that the robot must turn to face the target
     * @param currentPos the robot's current position, in meters
     * @param targetPos the position of the target that is to be hit, in meters
     * @return angle at which the robot will be facing the target
     */
    public static double getAngleToTarget(VectorF currentPos, VectorF targetPos){
        double changeInX = currentPos.get(0) - targetPos.get(0);
        double changeInY = currentPos.get(1) - targetPos.get(1);

        double angle = Math.tan(changeInY/changeInX);
        return  angle;
    }

    /**
     * Calculates the velocity required to launch something at a target position
     * @param launchPos the position that the object is launched at, in meters
     * @param targetPos the position of the target that is to be hit, in meters
     * @param launchAngle the angle of the object leaving the launcher, in degrees
     * @return velocity required to hit the target
     */
    public static double getVelocityToTarget(VectorF launchPos, VectorF targetPos, double launchAngle){
        double changeInX = FieldMap.getGroundDistanceBetweenTwoPoints(launchPos, targetPos);
        double changeInY = FieldMap.getHeightDifferenceBetweenTwoPoints(launchPos, targetPos);

        double velocity = Math.sqrt(((0.5 * g) * Math.pow(changeInX / Math.cos(launchAngle), 2)) / (changeInY + (Math.tan(launchAngle) * changeInX)));
        return velocity;
    }

    /**
     * Calculates the angular velocity for the given tangential velocity
     * @param tangentialVelocity tangential velocity to be converted in meters/second
     * @param wheelRadius radius of the wheel in meters
     * @return angular velocity in radians/second
     */
    public static double velocityToAngularVelocity(double tangentialVelocity, double wheelRadius){
        double angularVelocity = tangentialVelocity / wheelRadius;
        return  angularVelocity;
    }
}
