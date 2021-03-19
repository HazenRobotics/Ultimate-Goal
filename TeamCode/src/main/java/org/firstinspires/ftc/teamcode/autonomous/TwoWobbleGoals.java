package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;

public class TwoWobbleGoals extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private TensorFlowUtil.Stack stack;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotTechnicolorRR(hardwareMap, this);
        robot.setPosition(new Pose2d(-61, -48));
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        robot.tfod.initTensorFlow();

        waitForStart();

        //Detect stack
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-42, -40)).build());
        robot.tfod.runStackDetection(10000);
        stack = robot.tfod.getStack();

        //Move wobble goal to correct zone
        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-10, -60, Math.toRadians(180)), 0).build());
        } else if(stack == TensorFlowUtil.Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(14, -36, Math.toRadians(180)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(180)), 0).build());
        }

        Trajectory powershotTrajectory = robot.trajectoryBuilder()
                .lineToConstantHeading(new Vector2d(-42, -30))
                .splineToConstantHeading(new Vector2d(-10, -22.5), 0)
                .addDisplacementMarker(() -> {
                    robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT);
                })
                .lineTo(new Vector2d(-10, -17))
                .addDisplacementMarker(() -> {
                    robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT);
                })
                .lineTo(new Vector2d(-10, -9.5))
                .addDisplacementMarker(() -> {
                    robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT);
                })
                .build();

        //Drop wobble goal TODO: create method in robot class for this
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        sleep(1000);

        robot.driveAsync(powershotTrajectory);

        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LIFTED, 0.6);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        robot.drive.waitForIdle();

        //Shoot powershot targets

        /*robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-42, -30)).splineToConstantHeading(new Vector2d(-10, -22.5), 0).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-10, -17)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT);
        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-10, -9.5)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT);*/

        //Pick up 2nd wobble goal
        robot.drive(robot.trajectoryBuilder()
                .addDisplacementMarker(() -> {
                    robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6);
                    robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
                })
                .splineToLinearHeading(new Pose2d(-54, -20, 0), 0)
                .build());

        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LIFTED, 0.6);

        //Move wobble goal to correct zone (slightly to the left)
        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-10, -60, Math.toRadians(180)), 0).build());
        } else if(stack == TensorFlowUtil.Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(14, -36, Math.toRadians(180)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(38, -60, Math.toRadians(180)), 0).build());
        }

        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        sleep(1000);

        robot.driveAsync(robot.trajectoryBuilder().lineTo(new Vector2d(12, -18.5)).build());

        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LIFTED, 0.6);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        robot.drive.waitForIdle();


        //Return to center line

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
