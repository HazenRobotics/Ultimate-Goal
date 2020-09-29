package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class sets up and holds methods for running a mecanum drive
 */
public class MecanumDrive implements Drive {
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;

    /**
     * Creates a MechanumDrive with default names for the wheels
     * @param hw robot's hardware map
     */
    public MecanumDrive(HardwareMap hw){
        setUpMotors(hw, "frontRightWheel", "frontLeftWheel", "backRightWheel", "backLeftWheel");
    }

    /**
     * Creates a MechanumDrive with specified names for the wheels
     * @param hw robot's hardware map
     * @param frontRightMotorName name of front right motor in the hardware map
     * @param frontLeftMotorName name of front left motor in the hardware map
     * @param backRightMotorName name of back right motor in the hardware map
     * @param backLeftMotorName name of back left motor in the hardware map
     */
    public MecanumDrive(HardwareMap hw, String frontRightMotorName, String frontLeftMotorName, String backRightMotorName, String backLeftMotorName){
        setUpMotors(hw, frontRightMotorName, frontLeftMotorName, backRightMotorName, backLeftMotorName);
    }

    /**
     * Sets up motors from the hardware map
     * @param hw robot's hardware map
     * @param frontRightMotorName name of front right motor in the hardware map
     * @param frontLeftMotorName name of front left motor in the hardware map
     * @param backRightMotorName name of back right motor in the hardware map
     * @param backLeftMotorName name of back left motor in the hardware map
     */
    private void setUpMotors(HardwareMap hw, String frontRightMotorName, String frontLeftMotorName, String backRightMotorName, String backLeftMotorName) {
        frontRightMotor = hw.dcMotor.get(frontRightMotorName);
        frontLeftMotor = hw.dcMotor.get(frontLeftMotorName);
        backRightMotor = hw.dcMotor.get(backRightMotorName);
        backLeftMotor = hw.dcMotor.get(backLeftMotorName);
    }

    /**
     * Moves the robot forward and backward.
     * @param power power at which to run the motors (+ is forward, - is backwards)
     */
    @Override
    public void move(double power){
        drive(power, 0, 0);
    }

    /**
     * Turns the robot on the spot
     * @param power power at which to run the motors (+ is a right turn, - is a left turn)
     */
    @Override
    public void turn(double power) {
        drive(0, power, 0);
    }

    /**
     * Makes the robot strafe to the left or the right
     * @param power power at which to strafe (+ is to the right, - is to the left)
     */
    public void strafe(double power){
        drive(0, 0, power);
    }

    /**
     * Sets power to the wheel motors
     *
     * @param drive  power for forward and back motion
     * @param strafe power for left and right robot
     * @param rotate power for rotating the robot
     */
    public void drive( double drive, double strafe, double rotate ) {

        // You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe + rotate;
        double backLeftPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        setMotorPower( frontLeftPower, frontRightPower, backLeftPower, backRightPower );
    }

    /**
     * Sets specified power to the motors
     * @param frontRightPower power at which to run the front right motor.
     * @param frontLeftPower power at which to run the front left motor.
     * @param backRightPower power at which to run the back right motor.
     * @param backLeftPower power at which to run the back left motor.
     */
    private void setMotorPower(double frontRightPower, double frontLeftPower, double backRightPower, double backLeftPower){
        frontRightMotor.setPower(frontRightPower);
        frontLeftMotor.setPower(frontLeftPower);
        backRightMotor.setPower(backRightPower);
        backLeftMotor.setPower(backLeftPower);
    }


}
