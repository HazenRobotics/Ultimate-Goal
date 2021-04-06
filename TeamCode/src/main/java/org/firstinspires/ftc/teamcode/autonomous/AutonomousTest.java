package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotWood;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="AutonomousTest", group="autonomous")
//@disabled
public class AutonomousTest extends LinearOpMode
{
    //Robot robot;
    RobotWood robotWood;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        //robot = new RobotWood(hardwareMap, this);
        //robotWood = new RobotWood(hardwareMap, this);

        robotWood.tensorFlowUtil.initTensorFlow();

        waitForStart();

        //==========================================================================================
        //Official Start

        robotWood.tensorFlowUtil.runStackDetection(20000);



    }


}

