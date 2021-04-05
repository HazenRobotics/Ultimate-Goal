package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

@Autonomous(name="Two Wobble Goals", group="Competition" )
public class TwoWobbleGoals extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private TensorFlowUtil.Stack stack;

    private final boolean pickUpRing = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createDefaultMatchLogFileName( this.getClass().getName() );
        // otherwise do Robot.createDefaultMatchLogFileName( "TeleOpTechnicolor" );

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playStartup();

        robot.setPosition(new Pose2d(-62, -48));
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        robot.tfod.initTensorFlow();
 
        telemetry.addLine("Initialization Complete");
        telemetry.update();

        while(!isStarted()) {
            if(isStopRequested()) {
                robot.tfod.deactivateTensorFlow();
                if(Vuforia.getInstance().isRunning())
                    Vuforia.getInstance().close();
            }
        }

        waitForStart();

        //Detect stack
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-52, -40)).build());
        robot.tfod.runStackDetection(115);
        stack = robot.tfod.getStack();

        //shoot
        robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-13, -9, 0), 0).build());
        robot.ringShooter.setFlyWheelMotorVelocity(9.25, AngleUnit.RADIANS);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive.waitForIdle();
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(-13, -16, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(-13, -23.5, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();

        //if there is one ring in the stack, pick up the ring and shoot
        if(stack == TensorFlowUtil.Stack.SINGLE || stack == TensorFlowUtil.Stack.QUAD && pickUpRing) {
            robot.ringShooter.setIntakeMotorPower(0.8);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-10, -36)).build());
            robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
            robot.drive.waitForIdle();
            robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-24, -36)).build());
            sleep(500);
            robot.ringShooter.setIntakeMotorPower(0);
            if(stack == TensorFlowUtil.Stack.QUAD) {
                robot.ringShooter.launchRingAngularVelocity(10, false, false);
                robot.ringShooter.launchRingAngularVelocity(10, false, false);
            }
            robot.ringShooter.launchRingAngularVelocity(10, true, false);
        }

        //Move wobble goal to correct zone
        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-15, -60, Math.toRadians(182)), 0).build());
        } else if(stack == TensorFlowUtil.Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(12, -36, Math.toRadians(182)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(33, -58, Math.toRadians(182)), 0).build());
            //robot.setPosition(new Pose2d(33, -56));
            // off - puts wobble goal in center of square (4 rngs)
        }



        //Drop wobble goal TODO: create method in robot class for this
        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6, 700);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        sleep(500);


        //Pick up 2nd wobble goal
        robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-16, -30, 0), 90).build());
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 700);
        robot.drive.waitForIdle();
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LOWERED, 0.6, 500);
        if(stack == TensorFlowUtil.Stack.QUAD) {
            robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(-30, -30, Math.toRadians(10))).build());
        }
        else {
            robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(-30, -30, Math.toRadians(6))).build());
        }
        sleep(300);

        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
        sleep(1000);
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 1.0, 800);

        //Move wobble goal to correct zone (slightly to the left or back if single)
        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-15, -53, Math.toRadians(180)), 0).build());
        } else if(stack == TensorFlowUtil.Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(0, -36, Math.toRadians(177)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(33, -53, Math.toRadians(175)), 0).build());
        }

        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6, 700);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        sleep(500);

        //park
        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.driveAsync(robot.trajectoryBuilder().strafeRight(8).splineToConstantHeading( new Vector2d(10, -36), 0).build());
        }
        else {
            robot.driveAsync(robot.trajectoryBuilder().lineTo(new Vector2d(8, -36)).build());

        }

        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 800);
        robot.drive.waitForIdle();


        //Return to center line

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
