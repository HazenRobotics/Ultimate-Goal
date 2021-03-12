package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil.Stack;

public class ComplexAuto extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private Stack stack;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotTechnicolorRR(hardwareMap);
        robot.setPosition(new Pose2d(-60, -48));

        waitForStart();

        //Detect stack
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-36, -48)).build());
        robot.tfod.runStackDetection(1);
        stack = robot.tfod.getStack();


        //Shoot powershot targets
        robot.drive(robot.trajectoryBuilder().splineToConstantHeading(new Vector2d(-10, -18.5), 0).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-10, -11)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-10, -3.5)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT);

        //Move wobble goal to correct zone
        if(stack == Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToSplineHeading(new Pose2d(0, -60, Math.toRadians(180)), 0).build());
        } else if(stack == Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToSplineHeading(new Pose2d(24, -36, Math.toRadians(180)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToSplineHeading(new Pose2d(48, -60, Math.toRadians(180)), 0).build());
        }
        //Drop wobble goal TODO: create method in robot class for this
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.5);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LIFTED, 0.5);

        //Return to center line
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(12, -18.5)).build());

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
