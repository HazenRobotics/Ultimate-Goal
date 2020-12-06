package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.drives.*;
import org.firstinspires.ftc.teamcode.*;

public class Tracking {

    BNO055IMU gyro;
    MecanumDrive mecanumDrive;

    public Tracking( MecanumDrive mDrv, HardwareMap hw ) {

        mecanumDrive = mDrv;

        gyro = hw.get( BNO055IMU.class, "imu" );
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        gyro.initialize( parameters );

        /*
        // make sure the imu gyro is calibrated before continuing.
        while ( !gyro.isGyroCalibrated() )
            sleep( 50 );
        */

    }

    /**
     * returns the Longitudinal (forward/backward) distance on the encoders from where it started
     * @return mecanumDrive.getBackLeftPosition();
     */
    public int getLongitudinalPosition() {
        return mecanumDrive.getBackLeftPosition();
    }

    /**
     * returns the Lateral (left/right) distance on the encoders from where it started
     * @return mecanumDrive.getBackLeftPosition();
     */
    public int getLateralPosition() {
        return mecanumDrive.getFrontLeftPosition();
    }


    public Position getGyroPosition() {
        return gyro.getPosition();
    }

    public double getGyroX() {
        return gyro.getPosition().x;
    }

    public double getGyroY() {
        return gyro.getPosition().y;
    }

    public double getGyroZ() {
        return gyro.getPosition().z;
    }

    public DistanceUnit getGyro() {
        return gyro.getPosition().unit;
    }

    /**
     * will sleep the robot for [millis] milliseconds
     * @param millis
     */
    public void sleep( long millis ) {
        long startTime = System.currentTimeMillis();
        while( System.currentTimeMillis() < startTime + millis );
    }






}
