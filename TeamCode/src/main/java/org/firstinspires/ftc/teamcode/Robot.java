package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;

/**
 * This class sets up and manages a robot
 */
public class Robot {
    HardwareMap hardwareMap;

    Servo leftPusherServo;
    Servo rightPusherServo;

    RingShooter shooter;

    MecanumDrive drive;


    /**
     * Creates a Robot
     * @param hw robot's hardware map
     */
    public Robot( HardwareMap hw ) {
        this.hardwareMap = hw;

        leftPusherServo = hardwareMap.servo.get( "leftPusherServo" );
        rightPusherServo = hardwareMap.servo.get( "rightPusherServo" );

        shooter = new RingShooter(hw);

        drive = new MecanumDrive(hw);
    }

}
