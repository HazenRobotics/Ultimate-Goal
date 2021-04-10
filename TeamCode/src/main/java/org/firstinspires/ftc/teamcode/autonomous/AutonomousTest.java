package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name="AutonomousTest", group="autonomous")
//@disabled
public class AutonomousTest extends LinearOpMode {

        private RobotTechnicolorRR robot;
        private TensorFlowUtil.Stack stack;

        double initialTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playStartup();

        //robot.setPosition(new Pose2d(-61.125, -41.875));
        robot.setPosition(new Pose2d(9.375, -39.125));

        telemetry.addLine("Init Finished");
        telemetry.update();

        Robot.writeToMatchFile( "Init Finished", true );

        waitForStart();

        //==========================================================================================
        //Official Start

        testQuadShooting();

    }

    private void testQuadShooting() {

        //Thread driveThread = new Thread()

        // make sure to set the initial position to (9.375, -39.125) i.e:
        // robot.setPosition(new Pose2d(9.375, -39.125));

        double ringPosX = -35.375;

        initialTime = this.getRuntime();

        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(9.375, ringPosX)).build());

        stack = TensorFlowUtil.Stack.QUAD;

        robot.ringShooter.setIntakeMotorPower(1);
        robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
        robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-7, ringPosX)).build());
        robot.drive.waitForIdle();

        robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-17, ringPosX)).build()); // pick up ring 1
        robot.drive.waitForIdle();
        logAndPrint( this.getRuntime() - initialTime + " shoot ring 1" );
        robot.ringShooter.launchRingAngularVelocity(10.075, false, 250);
        if (stack == TensorFlowUtil.Stack.QUAD) {
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-21, ringPosX)).build()); // pick up ring 2
            logAndPrint( this.getRuntime() - initialTime + " shoot ring 2" );
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.1, false, 0);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-26, ringPosX)).build()); // pick up ring 3
            logAndPrint( this.getRuntime() - initialTime + " shoot ring 3" );
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-30, ringPosX)).build()); // pick up ring 4
            logAndPrint( this.getRuntime() - initialTime + " shoot ring 4" );
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);
            robot.ringShooter.launchRingAngularVelocity(10.35, false, 250);
        }
        logAndPrint( this.getRuntime() - initialTime + " set powers zero" );
        robot.ringShooter.setFlyWheelMotorPower(0);
        robot.ringShooter.setIntakeMotorPower(0);

    }



    public void logAndPrint( String text ) {
        Robot.writeToMatchFile( text, false );
        addLine(text);
    }

    public void addLine( String telemetryText ) {
        telemetry.addLine( telemetryText );
        telemetry.update();
    }

    private void driveToCenter( boolean goBack ) {
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(0, 0)).build());

        if( goBack ) {
            double delay = 2 * 1000;
            double waitUntil = getRuntime() + (double) (delay / 1000);
            while (getRuntime() < waitUntil) ;

            robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-61.125, -41.875)).build());
        }
    }


}

