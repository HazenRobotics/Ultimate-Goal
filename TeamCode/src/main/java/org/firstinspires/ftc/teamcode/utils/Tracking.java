package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.drives.*;

public class Tracking {

    BNO055IMU gyro;
    MecanumDrive mecanumDrive;
    Orientation angles;
    Acceleration gravity;

    // double P = 0.0135, I = 0.02025, D = 0;
    GeneralPID gyroPID = new GeneralPID( new PIDCoefficients( 0.1, 0, 0 ) );
    GeneralPID drivePID = new GeneralPID( new PIDCoefficients( 0, 0, 0 ) );
    GeneralPID strafePID = new GeneralPID( new PIDCoefficients( 0, 0, 0 ) );

    final static double PULSES_PER_REVOLUTION = 250;
    final static double GEAR_RATIO = 0.25;

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


    /**
     * returns the Longitudinal (forward/backward) distance on the encoders from where it started
     * @return mecanumDrive.getBackLeftPosition();
     */
    public float getXPosition() {
        return getLateralPosition();
    }

    /**
     * returns the Lateral (left/right) distance on the encoders from where it started
     * @return mecanumDrive.getBackLeftPosition();
     */
    public float getYPosition() {
        return getLongitudinalPosition();
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



    /**
     *
     * @param distanceToTravel the distance to move in inches
     * @param circumference the circumference of the wheel that has the encoder
     * @return totalTicks - the amount of ticks to move forward
     */
    public static int convertDistTicks( double distanceToTravel, double circumference )
    {
        double revolutions = distanceToTravel / circumference;
        int totalTicks = (int) Math.round( (revolutions * PULSES_PER_REVOLUTION) / GEAR_RATIO );

        return totalTicks;
    }
    public static int convertTicksDist( double ticksToTravel, double circumference )
    {
        double calculations = ticksToTravel * circumference * GEAR_RATIO;
        int totalDistance = (int) Math.round( calculations / PULSES_PER_REVOLUTION );

        return totalDistance;
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

    public float getGyroHeading() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public float getGyroRoll() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.secondAngle;
    }

    public float getGyroPitch() {
        angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.thirdAngle;
    }

    public float get360GyroHeading() {
        return (-getGyroHeading() + 360) % 360;
    }

    public double gyroPID( double targetAngle ) {
        return gyroPID.getPID( targetAngle, -getGyroHeading() );
    }

    public double drivePID( double targetDistance ) {
        return drivePID.getPID( targetDistance, getLateralPosition() );
    }

    public double strafePID( double targetDistance ) {
        return strafePID.getPID( targetDistance, getLongitudinalPosition() );
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
