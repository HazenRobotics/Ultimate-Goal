package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.RobotWood;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="MainAutonomous", group="autonomous")
//@disabled
public class MainAutonomous extends LinearOpMode
{

    RobotWood robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotWood(hardwareMap, this);
        robot.driveTrain = new MecanumDrive(hardwareMap);
        //robot.tensorFlowUtil.initTensorFlow(hardwareMap);

        //telemetry.setAutoClear( false );

        robot.writeToDefaultFile( "******** Init Finished ********", false, true );

        telemetry.addLine("init finished");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start


        robot.writeToDefaultFile( "******** Main Class Started ********", true, true );


        /*
        // add ordered list of Main PLan here:


        */

        /*
        //do all the driving methods here


        */

        robot.tensorFlowUtil.runStackDetection( 10000 );
        //robot.dropOffGoal();

    }



}

