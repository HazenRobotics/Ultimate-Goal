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
    Acceleration gravity;

    public Tracking( MecanumDrive mDrv, HardwareMap hw ) {

        mecanumDrive = mDrv;

        initGyro(hw);

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

    public void initGyro( HardwareMap hw ) {
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

    public double getGyroXVelocity() {
        return gyro.getVelocity().xVeloc;
    }

    public double getGyroYVelocity() {
        return gyro.getVelocity().yVeloc;
    }

    public double getGyroZVelocity() {
        return gyro.getVelocity().zVeloc;
    }

    public float get360GyroHeading() {
        return getGyroHeading();
    }

    public float getGyroHeading() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
        //return formatAngle(angles.angleUnit, angles.firstAngle);
    }

    public float getGyroRoll() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
        //return formatAngle(angles.angleUnit, angles.secondAngle);
    }

    public float getGyroPitch() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    public float getNewGyroHeading()
    {
        return (-getGyroHeading() + 360) % 360;
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
