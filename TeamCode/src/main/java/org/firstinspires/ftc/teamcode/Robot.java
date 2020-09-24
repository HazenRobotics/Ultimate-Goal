package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    HardwareMap hardwareMap;

    DcMotor leftMotor;
    DcMotor rightMotor;

    public Robot(HardwareMap hw){
        this.hardwareMap = hw;
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
    }

    /**
     * Moves the robot forwards or backwards
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is forwards, - is backwards</i>
     *
     */
    public void move(double power){
        setMotorPower(power, power);
    }

    /**
     * Turns the robot to the left or the right
     * @param power power at which to run the motors.
     *              <br><i>NOTE: + is to the right, - is to the left</i>
     */
    public void turn(double power){
        setMotorPower(power, -power);
    }


    // "raw" methods


    /*


    flywheels
    intake
    servos - pushers

    item names




     */


    /**
     * Sets power to the motors
     * @param leftPower power at which to run the left motor
     * @param rightPower power at which to run the right motor
     */
    private void setMotorPower(double leftPower, double rightPower){
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

}
