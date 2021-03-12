package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;

/**
 * This class sets up and holds methods for using the ring shooter mechanism
 * The ring shooter mechanism is the ______ that does _______
 */
public class RingShooterWood {

    public DcMotorEx leftFlyWheelMotor;
    public DcMotorEx rightFlyWheelMotor;

    public Servo pusher;
    private double pushedPosition = 0.5;
    private double retractedPosition = 0.25;

    private double flyWheelRadius;
    private static double launchAngle = 35; //degrees
    private double currentIntakePower;


    /**
     * Creates a RingShooter with default names for the motors
     *
     * @param hw robot's hardware map
     */
    public RingShooterWood(HardwareMap hw, double flyWheelRadius, double pushedPosition, double retractedPosition) {
        this(hw, "leftFlyWheel", "rightFlyWheel", "pusher", flyWheelRadius, pushedPosition, retractedPosition);
    }

    /**
     * Creates a RingShooter with specified names for the motors
     *
     * @param hw                robot's hardware map
     * @param leftFlyWheelName  name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    public RingShooterWood(HardwareMap hw, String leftFlyWheelName, String rightFlyWheelName, String pusherName, double flyWheelRadius, double pushedPosition, double retractedPosition) {
        setUpHardware(hw, leftFlyWheelName, rightFlyWheelName, pusherName);
        this.flyWheelRadius = flyWheelRadius;
        this.pushedPosition = pushedPosition;
        this.retractedPosition = retractedPosition;
        MotorConfigurationType motorConfigurationType = leftFlyWheelMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        leftFlyWheelMotor.setMotorType(motorConfigurationType);
        motorConfigurationType = rightFlyWheelMotor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
        rightFlyWheelMotor.setMotorType(motorConfigurationType);
        leftFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets up motors from the hardware map
     *
     * @param hw                robot's hardware map
     * @param leftFlyWheelName  name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    private void setUpHardware(HardwareMap hw, String leftFlyWheelName, String rightFlyWheelName, String pusherName) {

        leftFlyWheelMotor = hw.get(DcMotorEx.class, leftFlyWheelName);
        rightFlyWheelMotor = hw.get(DcMotorEx.class, rightFlyWheelName);

        //change these based on motor direction
        leftFlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFlyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        pusher = hw.servo.get(pusherName);
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
    public void launchRingVelocity(double velocity, DistanceUnit inputUnit) {
        setFlyWheelMotorVelocity(ShootingMath.velocityToAngularVelocity(inputUnit.toMeters(velocity), inputUnit.toMeters(flyWheelRadius)), AngleUnit.RADIANS);
        pushRing();
        setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     * Launches a ring stored in the magazine
     *
     * @param power power at which to run the motors when shooting the ring
     */
    public void launchRingPower(double power) {
        setFlyWheelMotorPower(power);
        pushRing();
        setFlyWheelMotorPower(0);
    }

    public void pushRing() {
        pusher.setPosition(pushedPosition);
        long currentTime = System.currentTimeMillis();
        while (System.currentTimeMillis() < currentTime + 2000);
        pusher.setPosition(retractedPosition);
    }

    public double getLaunchAngle() {
        return launchAngle;
    }

    public double getCurrentIntakePower() {
        return currentIntakePower;
    }

    public void setFlyWheelPID(PIDFCoefficients coeffs) {
        leftFlyWheelMotor.setVelocityPIDFCoefficients(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
        rightFlyWheelMotor.setVelocityPIDFCoefficients(coeffs.p, coeffs.i, coeffs.d, coeffs.f);
    }
}