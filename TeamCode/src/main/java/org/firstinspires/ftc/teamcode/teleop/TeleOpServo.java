package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLiftWood;
import org.firstinspires.ftc.teamcode.robots.RobotWood;

// TeleOp class for the new wooden robot
@TeleOp(name="teleopServo", group="teleop")
public class TeleOpServo extends OpMode {

    //Robot robot;
    RobotWood robot;

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap, this);

        telemetry.addLine("init finished");
        telemetry.update();
    }

    @Override
    public void loop() {


        robot.goalLift.claw.setPosition(gamepad1.right_stick_y);


        telemetry.addData("claw position", robot.goalLift.claw.getPosition());
        telemetry.update();

    }
}
