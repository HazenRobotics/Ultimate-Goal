package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;

/**
 * This class sets up and manages a robot
 */
public class Robot {
    HardwareMap hardwareMap;

    Servo leftPusherServo;
    Servo rightPusherServo;

    //drive type
    MecanumDrive drive;

    //mechanisms
    RingShooter shooter;
    GoalLift lift;


    /**
     * Creates a Robot
     * @param hw robot's hardware map
     */
    public Robot( HardwareMap hw ) {
        this.hardwareMap = hw;

        leftPusherServo = hardwareMap.servo.get( "leftPusherServo" );
        rightPusherServo = hardwareMap.servo.get( "rightPusherServo" );

        //drive
        drive = new MecanumDrive(hw);

        //mechanisms
        shooter = new RingShooter(hw);
        lift = new GoalLift(hw);
    }

}
