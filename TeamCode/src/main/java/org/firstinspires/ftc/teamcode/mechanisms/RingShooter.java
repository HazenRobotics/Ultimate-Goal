package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;

/**
 * This class sets up and holds methods for using the ring shooter mechanism
 * The ring shooter mechanism is the ______ that does _______
 */
public class RingShooter {

    private DcMotor intakeMotor;

    public DcMotorEx leftFlyWheelMotor;
    public DcMotorEx rightFlyWheelMotor;

    public Servo pusher;
    private double pushedPosition;
    private double retractedPosition;

    private double flyWheelRadius;
    private static double launchAngle = 35; // degrees

    private static final double RING_PUSH_TIME = 200; // milliseconds
    private static final double RING_RETRACT_TIME = 200;

    public enum PusherPosition {
        PUSHED,
        RETRACTED
    }

    private PusherPosition pusherPosition;

    /**
     * Creates a RingShooter with default names for the motors
     *
     * @param hw robot's hardware map
     */
    public RingShooter(HardwareMap hw, double flyWheelRadius, double pushedPosition, double retractedPosition, boolean reverseMotorDirections) {
        this(hw, "intake", "leftFlyWheel", "rightFlyWheel", "pusher", flyWheelRadius, pushedPosition, retractedPosition, reverseMotorDirections);
    }

    /**
     * Creates a RingShooter with specified names for the motors
     *
     * @param hw                robot's hardware map
     * @param intakeMotorName   name of intake motor in the hardware map
     * @param leftFlyWheelName  name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    public RingShooter(HardwareMap hw, String intakeMotorName, String leftFlyWheelName, String rightFlyWheelName,
                       String pusherName, double flyWheelRadius, double pushedPosition, double retractedPosition, boolean reverseMotorDirections ) {

        setUpHardware(hw, intakeMotorName, leftFlyWheelName, rightFlyWheelName, pusherName, reverseMotorDirections);

        this.flyWheelRadius = flyWheelRadius;

        this.pushedPosition = pushedPosition;
        this.retractedPosition = retractedPosition;
    }

    /**
     * Sets up motors from the hardware map
     *
     * @param hw                robot's hardware map
     * @param intakeMotorName   name of intake motor in the hardware map
     * @param leftFlyWheelName  name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    private void setUpHardware(HardwareMap hw, String intakeMotorName, String leftFlyWheelName, String rightFlyWheelName, String pusherName, boolean reverseMotorDirections ) {

        intakeMotor = hw.dcMotor.get(intakeMotorName);

        leftFlyWheelMotor = hw.get(DcMotorEx.class, leftFlyWheelName);
        rightFlyWheelMotor = hw.get(DcMotorEx.class, rightFlyWheelName);

        //change these based on motor direction
        leftFlyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        if( reverseMotorDirections ) {
            leftFlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightFlyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        pusher = hw.servo.get(pusherName);
    }

    /**
     * Sets power to the intake motor
     *
     * @param power power at which to run the intake motor
     */
    public void setIntakeMotorPower(double power) {
        intakeMotor.setPower(power);
    }

    /**
     * Sets power to the fly wheel motors
     *
     * @param power power at which to run the flywheels
     */
    public void setFlyWheelMotorPower(double power) {

        leftFlyWheelMotor.setPower(power);
        rightFlyWheelMotor.setPower(power);
    }

    /**
     * Sets velocity for the motor to run at
     *
     * @param velocity  target velocity
     * @param angleUnit unit in which the input velocity is given, in units/second
     */
    public void setFlyWheelMotorVelocity(double velocity, AngleUnit angleUnit) {
        leftFlyWheelMotor.setVelocity(velocity, angleUnit);
        rightFlyWheelMotor.setVelocity(velocity, angleUnit);
    }

    /**
     * Launches a ring stored in the magazine
     *
     * @param velocity  velocity at which to launch the ring
     * @param inputUnit unit used when inputting the velocity, in units/second
     */
    public void launchRingAngularVelocity(double velocity, DistanceUnit inputUnit) {
        setFlyWheelMotorVelocity(ShootingMath.velocityToAngularVelocity(inputUnit.toMeters(velocity), inputUnit.toMeters(flyWheelRadius)), AngleUnit.RADIANS);
        pushRingTime();
        setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     * Launches a ring stored in the magazine
     *
     * @param velocity  velocity at which to launch the ring
     * @param setPowerZero set power to zero afterwards
     * @param inputUnit unit used when inputting the velocity, in units/second
     */
    public void launchRingAngularVelocity(double velocity, boolean setPowerZero, DistanceUnit inputUnit) {
        setFlyWheelMotorVelocity(ShootingMath.velocityToAngularVelocity(inputUnit.toMeters(velocity), inputUnit.toMeters(flyWheelRadius)), AngleUnit.RADIANS);
        pushRingTime();
        if( setPowerZero )
            setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     *
     * @param omega to shoot with in rad/s
     * @param setPowerZero set motor power zero afterwards
     * @param speedUpTime wait 500 milliseconds before shooting
     */
    public void launchRingAngularVelocity(double omega, boolean setPowerZero, boolean speedUpTime) {
        setFlyWheelMotorVelocity(omega, AngleUnit.RADIANS);
        if (speedUpTime) {
            long currentTime = System.currentTimeMillis();
            while (System.currentTimeMillis() < currentTime + 500) ;
        }
        pushRingTime();
        if (setPowerZero)
            setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     *
     * @param omega to shoot with in rad/s
     * @param setPowerZero set motor power zero afterwards
     * @param speedUpTime time to wait before shooting in milliseconds
     */
    public void launchRingAngularVelocity(double omega, boolean setPowerZero, long speedUpTime) {
        setFlyWheelMotorVelocity(omega, AngleUnit.RADIANS);
        long currentTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < currentTime + speedUpTime);

        pushRingTime();
        if (setPowerZero)
            setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     *
     * @param omega to shoot with in rad/s
     * @param setPowerZero set motor power zero afterwards
     * @param speedUpTime time to wait before shooting in milliseconds
     */
    public void launchRingPositionAngularVelocity(double omega, boolean setPowerZero, long speedUpTime) {
        setFlyWheelMotorVelocity(omega, AngleUnit.RADIANS);
        long currentTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < currentTime + speedUpTime);

        pushRingPosition();
        if (setPowerZero)
            setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     * Launches a ring stored in the magazine
     *
     * @param power power at which to run the motors when shooting the ring
     */
    public void launchRingPower( double power ) {
        setFlyWheelMotorPower(power);
        pushRingTime();
        setFlyWheelMotorPower(0);
    }

    public void setNumericalPusherPosition( double position ) {
        pusher.setPosition( position );
    }

    public void setPusherPosition( PusherPosition position) {
        switch (position) {
            case PUSHED:
                pusherPosition = PusherPosition.PUSHED;
                pusher.setPosition(pushedPosition);
                break;
            case RETRACTED:
                pusherPosition = PusherPosition.RETRACTED;
                pusher.setPosition(retractedPosition);
                break;
        }
    }

    public void pushRingTime() {
        setPusherPosition(PusherPosition.PUSHED);
        SoundLibrary.playRandomPew(); // play a random pew sound
        long currentTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < currentTime + RING_PUSH_TIME);
        setPusherPosition(PusherPosition.RETRACTED);
        while (System.currentTimeMillis() < currentTime + RING_RETRACT_TIME);
    }

    public void pushRingPosition() {
        setPusherPosition(PusherPosition.PUSHED);
        SoundLibrary.playRandomPew(); // play a random pew sound
        long currentTime = System.currentTimeMillis();
        while (getPusherPosition() != pushedPosition);
        setPusherPosition(PusherPosition.RETRACTED);
        while (getPusherPosition() != retractedPosition);
    }

    public void pushRingTimeAsync() {
        new Thread(() -> pushRingTime()).start();
    }

    public void pushRingPositionAsync() {
        new Thread(() -> pushRingPosition()).start();
    }

    public double getPusherPosition() {
        return pusher.getPosition();
    }

    public PusherPosition getPusherLocation() {
        return pusherPosition;
    }

    public double getLaunchAngle() {
        return launchAngle;
    }

    public double getIntakePower() {
        return intakeMotor.getPower();
    }

    public double getFlyWheelPower( boolean getLeftFlyWheel ) {
        return getLeftFlyWheel ? leftFlyWheelMotor.getPower() : rightFlyWheelMotor.getVelocity();
    }

    public double getFlyWheelVelocity( boolean getLeftFlyWheel ) {
        return getLeftFlyWheel ? leftFlyWheelMotor.getVelocity( AngleUnit.RADIANS ) : rightFlyWheelMotor.getVelocity( AngleUnit.RADIANS );
    }

    public void setFlyWheelPID( PIDFCoefficients coeffs ) {

        leftFlyWheelMotor.setVelocityPIDFCoefficients(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
        rightFlyWheelMotor.setVelocityPIDFCoefficients(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
    }
}