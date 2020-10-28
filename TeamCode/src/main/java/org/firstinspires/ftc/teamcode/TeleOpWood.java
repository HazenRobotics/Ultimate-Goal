package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

// TeleOp class for the new wooden robot
@TeleOp(name="teleopWood", group="teleop")
public class TeleOpWood extends OpMode {

    Robot robot;

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap);
    }

    @Override
    public void loop() {

        // moves the robot • left stick - forwards/backwards - y axis, turning - right/left - x axis • right stick -  strafing - x axis
        ( (MecanumDrive)robot.drive ).drive( gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x );


    }
}
