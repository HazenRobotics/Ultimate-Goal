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

    DcMotor frontLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backLeftMotor;
    DcMotor backRightMotor;
    // Corban, we're gonna have 4 wheels (mecanum)


    public Robot(HardwareMap hw){
        this.hardwareMap = hw;

        intakeMotor = hardwareMap.dcMotor.get( "intakeMotor" );

        leftPusherServo = hardwareMap.servo.get( "leftPusherServo" );
        rightPusherServo = hardwareMap.servo.get( "rightPusherServo" );

        leftFlyWheelMotor = hardwareMap.dcMotor.get( "leftFlyWheelMotor" );
        rightFlyWheelMotor = hardwareMap.dcMotor.get( "rightFlyWheelMotor" );

        frontLeftMotor = hardwareMap.dcMotor.get( "frontLeftWheel" );
        frontRightMotor = hardwareMap.dcMotor.get( "frontRightWheel" );
        backLeftMotor = hardwareMap.dcMotor.get( "backLeftWheel" );
        backRightMotor = hardwareMap.dcMotor.get( "backRightWheel" );


    }

    /**
     * Moves the robot forwards or backwards
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is forwards, - is backwards</i>
     *
     */
    public void move(double power){
        setMotorPower( power, 0, 0 );
    }

    /**
     * Moves the robot forwards or backwards
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is forwards, - is backwards</i>
     *
     */
    public void strafe(double power){
        setMotorPower( 0, power, 0 );
    }

    /**
     * Turns the robot to the left or the right
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is to the right, - is to the left</i>
     */
    public void turn(double power){
        setMotorPower( 0, 0, power );
    }


    // "raw" methods



    /**
     * Sets power to the intake motor
     * @param power power at which to run the intake motor
     */
    private void setIntakeMotorPower( double power ) {
        intakeMotor.setPower( power );
    }

    /**
     * Sets power to the fly wheel motors
     * @param leftPower power at which to run the left fly wheel motor
     * @param rightPower power at which to run the right fly wheel motor
     */
    private void setFlyWheelMotorPower( double leftPower, double rightPower ) {
        leftFlyWheelMotor.setPower( leftPower );
        rightFlyWheelMotor.setPower( rightPower );
    }

    /**
     * Sets power to the wheel motors
     * @param drive power for forward and back motion
     * @param strafe power for left and right robot
     * @param rotate power for rotating the robot
     */
    private void setMotorPower( double drive, double strafe, double rotate ) {

        double frontLeftPower = drive + strafe + rotate;
        double frontRightPower = drive - strafe + rotate;
        double backLeftPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        frontLeftMotor.setPower( frontLeftPower );
        frontRightMotor.setPower( frontRightPower );
        backLeftMotor.setPower( backLeftPower );
        backRightMotor.setPower( backRightPower );

    }


}
