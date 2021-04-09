package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class AutonomousTest extends LinearOpMode {

        private RobotTechnicolorRR robot;
        private TensorFlowUtil.Stack stack;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playStartup();

        robot.setPosition(new Pose2d(-61.125, -41.875));

        telemetry.addLine("Init Finished");
        telemetry.update();

        Robot.writeToMatchDefaultFile( "Init Finished", true );

        waitForStart();

        //==========================================================================================
        //Official Start

        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(0, 0)).build());

        double delay = 1000;
        double waitUntil = getRuntime() + (double)(delay/1000);
        while( getRuntime() < waitUntil );

        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-61.125, -41.875)).build());
    }


}

