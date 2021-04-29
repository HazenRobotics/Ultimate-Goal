package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil.Stack;

@Autonomous(name="Complex Auto", group="Competition")
public class ComplexAuto extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private Stack stack;

    @Override
    public void runOpMode() throws InterruptedException {
    
        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        robot.setPosition(new Pose2d(-61, -48));
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        robot.tfod.initTensorFlow();

        telemetry.addLine("Initialization Complete");
        telemetry.update();
    
        Robot.writeToMatchFile( "Init Finished",  true );

        waitForStart();

        //Detect stack
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-52, -40)).build());
        robot.tfod.runStackDetection(115);
        stack = robot.tfod.getStack();


        //Shoot powershot targets
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-36, -36)).splineToConstantHeading(new Vector2d(-13, -25.5), 0).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, true);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-13, -20)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-13, -12.5)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);

        //Move wobble goal to correct zone
        if(stack == Stack.QUAD)
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(180)), 0).build()); // quad
        else if(stack == Stack.SINGLE)
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(14, -36, Math.toRadians(180)), 0).build()); // single
        else
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-10, -60, Math.toRadians(180)), 0).build()); // none


        //Drop wobble goal
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6, 500);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        sleep(1000);
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LIFTED, 0.6, 800);

        //Return to center line
        if(stack == Stack.QUAD)
            robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(0))).build());
        else
            robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(12, -24, Math.toRadians(180))).build());

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
