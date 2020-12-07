package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.drives.*;
import org.firstinspires.ftc.teamcode.*;

import java.util.Locale;

public class Tracking {

    BNO055IMU gyro;
    MecanumDrive mecanumDrive;
    Orientation angles;

    public Tracking( MecanumDrive mDrv, HardwareMap hw ) {

        mecanumDrive = mDrv;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        gyro = hw.get( BNO055IMU.class, "imu" );
        gyro.initialize(parameters);

        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

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



    public String getGyroHeading() {
        return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    public String getGyroRoll() {
        return formatAngle(angles.angleUnit, angles.secondAngle);
    }

    public String getGyroPitch() {
        return formatAngle(angles.angleUnit, angles.thirdAngle);
    }


    public DistanceUnit getGyro() {
        return gyro.getPosition().unit;
    }


    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format( Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
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
