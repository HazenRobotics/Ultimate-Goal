package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.ShootingMath;

/**
 * This class sets up and holds methods for using the ring shooter mechanism
 * The ring shooter mechanism is the ______ that does _______
 */
public class RingShooter {

    private DcMotor intakeMotor;

    public DcMotor leftFlyWheelMotor;
    public DcMotor rightFlyWheelMotor;

    public Servo pusher;

    private double flyWheelRadius;
    private double launchAngle;
    private double currentIntakePower;


    /**
     * Creates a RingShooter with default names for the motors
     * @param hw robot's hardware map
     */
    public RingShooter(HardwareMap hw, double flyWheelRadius){
        this( hw, "intake", "leftFlyWheel", "rightFlyWheel", "pusher", flyWheelRadius );
    }

    /**
     * Creates a RingShooter with specified names for the motors
     * @param hw robot's hardware map
     * @param intakeMotorName name of intake motor in the hardware map
     * @param leftFlyWheelName name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    public RingShooter( HardwareMap hw, String intakeMotorName, String leftFlyWheelName, String rightFlyWheelName, String pusherName, double flyWheelRadius) {
        setUpHardware( hw, intakeMotorName, leftFlyWheelName, rightFlyWheelName, pusherName );
        this.flyWheelRadius = flyWheelRadius;
    }

    /**
     * Sets up motors from the hardware map
     * @param hw robot's hardware map
     * @param intakeMotorName name of intake motor in the hardware map
     * @param leftFlyWheelName name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    private void setUpHardware( HardwareMap hw, String intakeMotorName, String leftFlyWheelName, String rightFlyWheelName, String pusherName ) {

        intakeMotor = hw.dcMotor.get( intakeMotorName );

        leftFlyWheelMotor = hw.dcMotor.get( leftFlyWheelName );
        rightFlyWheelMotor = hw.dcMotor.get( rightFlyWheelName );

        //change these based on motor direction
        leftFlyWheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFlyWheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher = hw.servo.get(pusherName);
    }

    /**
     * Sets power to the intake motor
     *
     * @param power power at which to run the intake motor
     */
    public void setIntakeMotorPower( double power ) {

        intakeMotor.setPower( power );
        currentIntakePower = power;
    }

    /**
     * Sets power to the fly wheel motors
     *
     * @param power power at which to run the flywheels
     */
    public void setFlyWheelMotorPower( double power) {
        leftFlyWheelMotor.setPower( power );
        rightFlyWheelMotor.setPower( power );
    }

    /**
     * Sets velocity for the motor to run at
     * @param velocity target velocity
     * @param  angleUnit unit in which the input velocity is given, in units/second
     */
    public void setFlyWheelMotorVelocity(double velocity, AngleUnit angleUnit){
        ((DcMotorEx)leftFlyWheelMotor).setVelocity(velocity, angleUnit);
        ((DcMotorEx)rightFlyWheelMotor).setVelocity(velocity, angleUnit);
    }

    /**
     * Launches a ring stored in the magazine
     * @param velocity velocity at which to launch the ring
     * @param inputUnit unit used when inputting the velocity, in units/second
     */
    public void launchRingVelocity(double velocity, DistanceUnit inputUnit) {
        setFlyWheelMotorVelocity(ShootingMath.velocityToAngularVelocity(inputUnit.toMeters(velocity), inputUnit.toMeters(flyWheelRadius)), AngleUnit.RADIANS);
        pushRing();
        setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
    }

    /**
     * Launches a ring stored in the magazine
     * @param power power at which to run the motors when shooting the ring
     */
    public void launchRingPower(double power) {
        setFlyWheelMotorPower(power);
        pushRing();
        setFlyWheelMotorPower(0);
    }

    private void pushRing() {
        pusher.setPosition(0.0);
        //Might need to put a pause here
        pusher.setPosition(0.3);
    }

    public double getLaunchAngle() {
        return launchAngle;
    }

    public double getCurrentIntakePower() {
        return  currentIntakePower;
    }

}
