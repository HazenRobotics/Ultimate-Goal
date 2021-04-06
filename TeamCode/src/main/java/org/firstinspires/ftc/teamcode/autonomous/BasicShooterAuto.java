package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;

@Autonomous(name="Basic Auto")
public class BasicShooterAuto extends LinearOpMode {

    private RobotTechnicolorRR robot;

    @Override
    public void runOpMode() throws InterruptedException {
    
        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        robot.setPosition(new Pose2d(-60, -48));

        Robot.writeToMatchDefaultFile( "Init Finished", true );
        
        waitForStart();



        //Shoot powershot targets
        robot.drive(robot.trajectoryBuilder().splineToConstantHeading(new Vector2d(-10, -22.5), 0).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true, true);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-10, -17)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, true, false);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-10, -9.5)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);



        //Return to center line
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(12, -18.5)).build());

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
