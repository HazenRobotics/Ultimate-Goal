package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * This class sets up and holds methods for using the ring shooter mechanism
 */
public class RingShooter {

    private DcMotor intakeMotor;

    private DcMotor leftFlyWheelMotor;
    private DcMotor rightFlyWheelMotor;


    /**
     * Creates a RingShooter with default names for the motors
     * @param hw robot's hardware map
     */
    public RingShooter(HardwareMap hw){
        setUpHardware( hw, "intakeMotor", "leftFlyWheelMotor", "rightFlyWheelMotor" );
    }

    /**
     * Creates a RingShooter with specified names for the motors
     * @param hw robot's hardware map
     * @param intakeMotorName name of intake motor in the hardware map
     * @param leftFlyWheelName name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    public RingShooter( HardwareMap hw, String intakeMotorName, String leftFlyWheelName, String rightFlyWheelName ) {
        setUpHardware( hw, intakeMotorName, leftFlyWheelName, rightFlyWheelName );
    }

    /**
     * Sets up motors from the hardware map
     * @param hw robot's hardware map
     * @param intakeMotorName name of intake motor in the hardware map
     * @param leftFlyWheelName name of left flywheel motor in the hardware map
     * @param rightFlyWheelName name of right flywheel motor in the hardware map
     */
    private void setUpHardware( HardwareMap hw, String intakeMotorName, String leftFlyWheelName, String rightFlyWheelName ) {

        intakeMotor = hw.dcMotor.get( intakeMotorName );

        leftFlyWheelMotor = hw.dcMotor.get( leftFlyWheelName );
        rightFlyWheelMotor = hw.dcMotor.get( rightFlyWheelName );
    }

    /**
     * Sets power to the intake motor
     *
     * @param power power at which to run the intake motor
     */
    public void setIntakeMotorPower( double power ) {

        intakeMotor.setPower( power );
    }

    /**
     * Sets power to the fly wheel motors
     *
     * @param power power at which to run the flywheels
     */
    public void setFlyWheelMotorPower( double power) {

        //the negative sign depends on how the robot is set up
        leftFlyWheelMotor.setPower( power );
        rightFlyWheelMotor.setPower( -power );
    }

}
