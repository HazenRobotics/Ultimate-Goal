package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.RobotWood;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

//@Autonomous(name="simpletest", group="SimpleTests") // uncomment this for it to appear in the DS app
//@disabled
public class SimpleTest extends LinearOpMode
{
    //Robot robot;
    //RobotWood robotWood;

    @Override
    public void runOpMode() throws InterruptedException
    {

        telemetry.addLine("init finished");
        telemetry.update();

        waitForStart();




        telemetry.addLine("program finished");
        telemetry.update();

        sleepRobot(1000);

    }

    public void sleepRobot(long delay)
    {
        long setTime = System.currentTimeMillis();

        while( (System.currentTimeMillis() - setTime)*1000 < (delay) && opModeIsActive()) {}

        telemetry.addData("Finished Sleep", "");
        telemetry.update();
    }
}

