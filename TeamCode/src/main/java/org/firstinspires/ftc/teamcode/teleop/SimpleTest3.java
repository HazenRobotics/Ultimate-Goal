package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.RobotWood;

@TeleOp(name="simpletest3", group="SimpleTests") // uncomment this for it to appear in the DS app
public class SimpleTest3 extends OpMode
{

    RobotWood robot;

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap, this);

        telemetry.addLine("init finished");
        telemetry.update();
    }

    @Override
    public void loop()
    {

        telemetry.addData("              Controls", "   ");
        telemetry.addData("Drive", "Gp1: left stick y (axis)");
        telemetry.addData("Strafe", "Gp1: left stick x (axis)");
        telemetry.addData("Rotate", "Gp1: right stick x (axis)");
        telemetry.addLine();






        telemetry.update();

    }
}