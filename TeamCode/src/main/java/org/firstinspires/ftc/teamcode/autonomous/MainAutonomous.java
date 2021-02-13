package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.RobotWood;
import org.firstinspires.ftc.teamcode.utils.TensorFlow;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="Main1Autonomous", group="autonomous")
//@disabled
public class MainAutonomous extends LinearOpMode
{

    RobotWood robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotWood(hardwareMap, this);
        robot.driveTrain = new MecanumDrive(hardwareMap);


        telemetry.addLine("init finished");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start

        /*
        // add ordered list of Main PLan here:


         */

        /*
        //do all the driving methods here


        */


        // recognition code
        //robot.getRingStack( TensorFlow.getStack() );
        robot.tensorFlowUtil.runStackDetection();






    }


}

