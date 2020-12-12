package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.RobotClapper;
import org.firstinspires.ftc.teamcode.RobotWood;
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
    public void runOpMode() throws InterruptedException
    {
        //robot = new RobotWood(hardwareMap, this);
        //robot.driveTrain = new MecanumDrive(hardwareMap);
        robotWood = new RobotWood(hardwareMap, this);
        robotWood.driveTrain = new MecanumDrive(hardwareMap);


        telemetry.addLine("init finished");
        telemetry.addLine("longitudinal position_0 = " + robotWood.tracker.getLongitudinalPosition() );
        telemetry.addLine("lateral position_0 = " + robotWood.tracker.getLateralPosition() );
        telemetry.update();

        waitForStart();

        //==========================================================================================
        //Official Start

        /*
        robotWood.driveDistance( 4, 0.5, true );

        telemetry.addLine( "driveDistance: 0.50 - completed" );
        telemetry.update();

        robotWood.sleep( 500 );
s
         */

        boolean testTwice = false;

        robotWood.driveTime( 500, .5, true );
        robotWood.rotateTime( 500, .5, true );

        //robotWood.driveDistance( 20, .5, true );

        telemetry.addLine("longitudinal position_1 = " + robotWood.tracker.getLongitudinalPosition() );
        telemetry.addLine("lateral position_1 = " + robotWood.tracker.getLateralPosition() );
        telemetry.update();

        if( testTwice ) {


            robotWood.sleep( 20 * 1000 );


            robotWood.driveDistance( 20, 0.25, true );

            telemetry.addLine( "longitudinal position_2 = " + robotWood.tracker.getLongitudinalPosition() );
            telemetry.addLine( "lateral position_2 = " + robotWood.tracker.getLateralPosition() );
            telemetry.update();
        }


        //robotWood.rotateDegrees( 270, -1/*, true*/ ); // should turn right 90 degrees

        //robotWood.rotateDegrees( 9, 0.6/*, true*/ ); // should turn right 90 degrees

    }


}

