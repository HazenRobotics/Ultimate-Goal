package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDrive {
    DcMotor frontRightMotor;
    DcMotor frontLeftMotor;
    DcMotor backRightMotor;
    DcMotor backLeftMotor;

    public MecanumDrive(HardwareMap hw){
        frontRightMotor = hw.dcMotor.get("frontRightMotor");
        frontLeftMotor = hw.dcMotor.get("frontLeftMotor");
        backRightMotor = hw.dcMotor.get("backRightMotor");
        backLeftMotor = hw.dcMotor.get("backLeftMotor");
    }

    /**
     * Moves the robot forward and backward.
     * @param power power at which to run the motors (+ is forward, - is backwards)
     */
    public void move(double power){
        setMotorPower(power, power, power, power);
    }

    /**
     * Turns the robot on the spot
     * @param power power at which to run the motors (+ is a right turn, - is a left turn)
     */
    public void turn(double power) {
        setMotorPower(-power, power, -power, power);
    }

    /**
     * Makes the robot strafe to the left or the right
     * @param power power at which to strafe (+ is to the right, - is to the left)
     */
    public void strafe(double power){

    }

    /**
     * Sets specified power to the motors.
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
