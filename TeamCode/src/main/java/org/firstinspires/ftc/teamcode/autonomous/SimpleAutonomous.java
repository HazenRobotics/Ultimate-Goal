package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotWood;
import org.firstinspires.ftc.teamcode.tests.PicturePIDTest;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="SimpleAutonomous", group="autonomous")
//@disabled
public class SimpleAutonomous extends LinearOpMode
{

    RobotWood robot;
    PicturePIDTest picTest;

    @Override
    public void runOpMode() throws InterruptedException
    {
        robot = new RobotWood(hardwareMap, this);
        robot.driveTrain = new MecanumDrive(hardwareMap);
        //robot.tensorFlowUtil.initTensorFlow(hardwareMap);

        //telemetry.setAutoClear( false );

        Robot.writeToDefaultFile( "******** Init Finished ********", false, true );

        telemetry.addLine("init finished");
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start

        Robot.writeToDefaultFile( "******** Main Class Started ********", true, true );

        robot.driveDistancePID( 24, 0.5, true );
        //robot.driveDistance( 15, 1, true );

        if( false ) {
            picTest = new PicturePIDTest();

            telemetry.addLine("Height :: " + picTest.getHeight());
            telemetry.addLine("Width :: " + picTest.getWidth());
            telemetry.update();
        }

        robot.sleepRobot2( 5*1000 );



/*


        robot.strafeDistancePID( 15, -0.5, true );



        robot.sleepRobot2( 3*1000 );

        robot.driveDistancePID( 10, -0.5, true );


        robot.strafeDistancePID( 10, 0.5, true );


 */

    }


}

