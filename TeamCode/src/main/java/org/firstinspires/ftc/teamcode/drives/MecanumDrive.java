package org.firstinspires.ftc.teamcode.drives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class sets up and holds methods for running a mecanum drive
 */
public class MecanumDrive extends FourWheelDrive {
    private DcMotor frontRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor backLeftMotor;
    /**
     * Creates a MechanumDrive with default names for the wheels
     * @param hw robot's hardware map
     */
    public MecanumDrive(HardwareMap hw){
        super(hw);
    }

    /**
     * Creates a MechanumDrive with specified names for the wheels
     * @param hw robot's hardware map
     * @param frontRightMotorName name of front right motor in the hardware map
     * @param frontLeftMotorName name of front left motor in the hardware map
     * @param backRightMotorName name of back right motor in the hardware map
     * @param backLeftMotorName name of back left motor in the hardware map
     */
    public MecanumDrive(HardwareMap hw, String frontLeftMotorName, String frontRightMotorName, String backLeftMotorName, String backRightMotorName){
        super(hw, frontRightMotorName, frontLeftMotorName, backRightMotorName, backLeftMotorName);
    }

    /**
     * Makes the robot strafe to the left or the right
     * @param power power at which to strafe (+ is to the right, - is to the left)
     */
    public void strafe(double power){
        drive(0, 0, power );
    }

    /**
     * Sets power to the wheel motors
     *
     * @param drive  power for forward and back motion
     * @param strafe power for left and right robot
     * @param rotate power for rotating the robot
     */
    public void drive( double drive, double rotate, double strafe ) {

        // You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower = drive + rotate + strafe;
        double frontRightPower = drive - rotate - strafe;
        double backLeftPower = drive - rotate - strafe;
        double backRightPower = drive + rotate + strafe;

        setMotorPower( frontLeftPower, frontRightPower, backLeftPower, backRightPower );
    }

}
