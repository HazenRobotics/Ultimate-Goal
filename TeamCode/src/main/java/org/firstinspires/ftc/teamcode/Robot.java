package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    HardwareMap hardwareMap;


    DcMotor intakeMotor;

    Servo leftPusherServo;
    Servo rightPusherServo;

    DcMotor leftFlyWheelMotor;
    DcMotor rightFlyWheelMotor;

    DcMotor frontLeftWheelMotor;
    DcMotor frontRightWheelMotor;
    DcMotor backLeftWheelMotor;
    DcMotor backRightWheelMotor;
    // Corban, we're gonna have 4 wheels (mecanum)


    public Robot( HardwareMap hw ) {
        this.hardwareMap = hw;

        intakeMotor = hardwareMap.dcMotor.get( "intakeMotor" );

        leftPusherServo = hardwareMap.servo.get( "leftPusherServo" );
        rightPusherServo = hardwareMap.servo.get( "rightPusherServo" );

        leftFlyWheelMotor = hardwareMap.dcMotor.get( "leftFlyWheelMotor" );
        rightFlyWheelMotor = hardwareMap.dcMotor.get( "rightFlyWheelMotor" );

        frontLeftWheelMotor = hardwareMap.dcMotor.get( "frontLeftWheel" );
        frontRightWheelMotor = hardwareMap.dcMotor.get( "frontRightWheel" );
        backLeftWheelMotor = hardwareMap.dcMotor.get( "backLeftWheel" );
        backRightWheelMotor = hardwareMap.dcMotor.get( "backRightWheel" );


    }

    /**
     * Moves the robot forwards or backwards
     *
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is forwards, - is backwards</i>
     */
    public void move( double power ) {
        mecanumDrive( power, 0, 0 );
    }

    /**
     * Moves the robot forwards or backwards
     *
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is forwards, - is backwards</i>
     */
    public void strafe( double power ) {
        mecanumDrive( 0, power, 0 );
    }

    /**
     * Turns the robot to the left or the right
     *
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is to the right, - is to the left</i>
     */
    public void turn( double power ) {

        mecanumDrive( 0, 0, power );
    }


    // "raw" methods


    /**
     * Sets power to the intake motor
     *
     * @param power power at which to run the intake motor
     */
    private void setIntakeMotorPower( double power ) {
        intakeMotor.setPower( power );
    }

    /**
     * Sets power to the fly wheel motors
     *
     * @param leftPower  power at which to run the left fly wheel motor
     * @param rightPower power at which to run the right fly wheel motor
     */

    private void setFlyWheelMotorPower( double leftPower, double rightPower ) {

        leftFlyWheelMotor.setPower( leftPower );
        rightFlyWheelMotor.setPower( rightPower );
    }

    /**
     * Sets power to the wheel motors
     *
     * @param drive  power for forward and back motion
     * @param strafe power for left and right robot
     * @param rotate power for rotating the robot
     */
    public void mecanumDrive( double drive, double strafe, double rotate ) {

        // You might have to play with the + or - depending on how your motors are installed
        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe + rotate;
        double backLeftPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        setWheelMotorPower( frontLeftPower, frontRightPower, backLeftPower, backRightPower );
    }


    /**
     * Sets power to the motors
     *
     * @param frontLeftPower power at which to run the front left motor
     * @param frontRightPower power at which to run the front right motor
     * @param backLeftPower power at which to run the back left motor
     * @param backRightPower power at which to run the back right motor
     */
    private void setWheelMotorPower( double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower ) {

        frontLeftWheelMotor.setPower( frontLeftPower );
        frontRightWheelMotor.setPower( frontRightPower );
        backLeftWheelMotor.setPower( backLeftPower );
        backRightWheelMotor.setPower( backRightPower );

    }

}
