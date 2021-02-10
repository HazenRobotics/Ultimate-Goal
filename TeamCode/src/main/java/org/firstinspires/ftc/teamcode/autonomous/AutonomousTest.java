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

    RobotWood robot;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotWood(hardwareMap, this);


        telemetry.addLine("init finished");
        telemetry.addLine("longitudinal position_0 = " + robot.tracker.getLongitudinalPosition() );
        telemetry.addLine("lateral position_0 = " + robot.tracker.getLateralPosition() );
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start

        /*
        robot.driveDistance( 4, 0.5, true );

        telemetry.addLine( "driveDistance: 0.50 - completed" );
        telemetry.update();

        robot.sleep( 500 );
s
         */

        boolean testTwice = false;

        //robot.driveTime( 500, .5, true );
        //robot.rotateTime( 1500, .5, true );

        robot.driveDistance( 12, 0.75 , true );
        //robot.strafeDistance( 12, 0.75 , true );
        //robot.rotateDegrees( 90, 0.75 );

        telemetry.addLine("longitudinal position_1 = " + robot.tracker.getLongitudinalPosition() );
        telemetry.addLine("lateral position_1 = " + robot.tracker.getLateralPosition() );
        telemetry.update();

        if( testTwice ) {

            robot.sleep( 20 * 1000 );


            robot.driveDistance( 20, 0.25, true );

            telemetry.addLine( "longitudinal position_2 = " + robot.tracker.getLongitudinalPosition() );
            telemetry.addLine( "lateral position_2 = " + robot.tracker.getLateralPosition() );
            telemetry.update();
        }


        //robot.rotateDegrees( 270, -1 /*, true*/ ); // should turn right 90 degrees

        //robot.rotateDegrees( 9, 0.6 /*, true*/ ); // should turn right 90 degrees

    }


}

