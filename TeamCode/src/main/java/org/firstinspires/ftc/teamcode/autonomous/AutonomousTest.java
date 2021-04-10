package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.robots.RobotWood;
import org.firstinspires.ftc.teamcode.drives.MecanumDrive;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="AutonomousTest", group="autonomous")
//@disabled
public class AutonomousTest extends LinearOpMode
{
    RobotTechnicolorRR robot;
    private TensorFlowUtil.Stack stack;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playStartup();

        robot.setPosition(new Pose2d(-62, -48));
        robot.tfod.initTensorFlow();

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        Robot.writeToMatchDefaultFile( "Init Finished", true );

        while(!isStarted()) {
            if(isStopRequested()) {
                robot.tfod.deactivateTensorFlow();
                if(Vuforia.getInstance().isRunning())
                    Vuforia.getInstance().close();
            }
        }

        waitForStart();

        //==========================================================================================
        //Official Start

        // starts at -62, -48
        // scans at -52, -40
        robot.tfod.runStackDetection(20000);



    }


}

