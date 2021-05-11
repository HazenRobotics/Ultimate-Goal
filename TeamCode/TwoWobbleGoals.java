package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
    private final boolean experimental = false;
    private final boolean alternateQuadRoute = true;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );
        
        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playStartup();

        robot.setPosition(new Pose2d(-61.125, -41.875));
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);
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

        //Detect stack
        detectStack();

        if( alternateQuadRoute && stack == TensorFlowUtil.Stack.QUAD ) {
            // move out of way of rings
            robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-45, -57)).build());

            // move wobble goal one to corner
            moveWobbleGoalOne();

            // drop off wobble goal 1
            dropWobbleGoal();

            // shoot the rings from lef tto right
            shootRings();

            // pick up the quad stack and shoot them in the high goal
            pickUpStackAndShoot();

        } else {

            //shoot
            shootRings();

            //if there is one ring in the stack, pick up the ring and shoot
            pickUpStackAndShoot();

            //Move wobble goal to correct zone
            moveWobbleGoalOne();

            //Drop wobble goal
            dropWobbleGoal();
        }

        //Pick up 2nd wobble goal
        pickUpWobbleGoalTwo();

        //Move wobble goal to correct zone (slightly to the left or back if single)
        moveWobbleGoalTwo();

        // Drop off 2nd wobble goal
        dropWobbleGoal();

        //park
        park();

        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 800);
        robot.drive.waitForIdle();


        //Return to center line

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    private void park() {

        if(stack == TensorFlowUtil.Stack.NONE)
            robot.driveAsync(robot.trajectoryBuilder().strafeRight(8).splineToConstantHeading( new Vector2d(10, -36), 0).build());
        else
            robot.driveAsync(robot.trajectoryBuilder().lineTo(new Vector2d(8, -36)).build());

    }

    private void moveWobbleGoalTwo() {

        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-15, -53, Math.toRadians(180)), 0).build());
        } else if(stack == TensorFlowUtil.Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(10, -36, Math.toRadians(177)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(33, -53, Math.toRadians(175)), 0).build());
        }
    }

    private void pickUpWobbleGoalTwo() {

        robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-16, -30, 0), 90).build());
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 700);
        robot.drive.waitForIdle();
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LOWERED, 0.6, 500);
        double radianSplice = stack == TensorFlowUtil.Stack.QUAD ? 10 : 6;
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(-25.625-1.5, -30.5, Math.toRadians(radianSplice))).build());

        sleep(300);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.CLOSED);

        sleep(1000);
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 1.0, 800);
    }

    private void dropWobbleGoal() {

        robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, 0.6, 700);
        robot.goalLift.setClawPosition(GoalLift.ClawPosition.OPEN);
        sleep(500);
    }

    private void moveWobbleGoalOne() {

        if(stack == TensorFlowUtil.Stack.NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-15, -60, Math.toRadians(182)), 0).build());
        } else if(stack == TensorFlowUtil.Stack.SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(20, -36, Math.toRadians(182)), 0).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(33, -58, Math.toRadians(182)), 0).build());
            //robot.setPosition(new Pose2d(33, -56));
            // off - puts wobble goal in center of square (4 rings)
        }
        robot.ringShooter.setIntakeMotorPower(0);
    }

    private void pickUpStackAndShoot() {
        if ((stack == TensorFlowUtil.Stack.SINGLE || stack == TensorFlowUtil.Stack.QUAD) && pickUpRing) {

            /* see you later
            double ringPosX = -35.375;

            robot.ringShooter.setIntakeMotorPower(1);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-10, ringPosX)).build());
            robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
            robot.drive.waitForIdle();

            if (!experimental) {

                robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-24, ringPosX)).build());
                sleep(500);
                robot.ringShooter.setIntakeMotorPower(0);
                if (stack == TensorFlowUtil.Stack.QUAD) {
                    robot.ringShooter.launchRingAngularVelocity(10, false, false);
                    robot.ringShooter.launchRingAngularVelocity(10, false, false);
                }
                robot.ringShooter.launchRingAngularVelocity(10, true, false);

            } else {

                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-30, ringPosX)).build());
                long currentTime = System.currentTimeMillis();
                while (System.currentTimeMillis() < currentTime + 200) ;
                if (stack == TensorFlowUtil.Stack.QUAD) {
                    robot.ringShooter.launchRingAngularVelocity(10, false, false);
                    robot.ringShooter.launchRingAngularVelocity(10.1, false, false);
                    robot.ringShooter.launchRingAngularVelocity(10.2, false, false);
                }
                robot.ringShooter.launchRingAngularVelocity(10.2, true, false);
                robot.drive.waitForIdle();
                robot.ringShooter.setIntakeMotorPower(0);

            }
        }

             */

            double ringPosX = -35.375;

            double initialTime = this.getRuntime();

            robot.ringShooter.setIntakeMotorPower(1);
            robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-7, ringPosX)).build());
            robot.drive.waitForIdle();

            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-17, ringPosX)).build()); // pick up ring 1
            robot.drive.waitForIdle();
            logAndPrint(this.getRuntime() - initialTime + " shoot ring 1");
            robot.ringShooter.launchRingAngularVelocity(10.075, false, 250);
            if (stack == TensorFlowUtil.Stack.QUAD) {
                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-21, ringPosX)).build()); // pick up ring 2
                logAndPrint(this.getRuntime() - initialTime + " shoot ring 2");
                robot.drive.waitForIdle();
                robot.ringShooter.launchRingAngularVelocity(10.1, false, 0);
                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-26, ringPosX)).build()); // pick up ring 3
                logAndPrint(this.getRuntime() - initialTime + " shoot ring 3");
                robot.drive.waitForIdle();
                robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);
                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-31, ringPosX)).build()); // pick up ring 4
                logAndPrint(this.getRuntime() - initialTime + " shoot ring 4");
                robot.drive.waitForIdle();
                robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);
                robot.ringShooter.launchRingAngularVelocity(10.3, false, 250);
            }
            logAndPrint(this.getRuntime() - initialTime + " set powers zero");
            robot.ringShooter.setFlyWheelMotorPower(0);
        }

    }

    public void logAndPrint( String text ) {
        Robot.writeToMatchDefaultFile( text, false );
        telemetry.addLine(text);
        telemetry.update();
    }

    private void shootRings() {

        double shootX = -13;

        robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(shootX, -4.875, 0), 0).build());
        robot.ringShooter.setFlyWheelMotorVelocity(9.75, AngleUnit.RADIANS);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive.waitForIdle();
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(shootX, -12.375, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(shootX, -19.875, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
    }

    private void detectStack() {
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-56, -41.875)).build());
        robot.tfod.runStackDetection(20000);
        stack = robot.tfod.getStack();
    }
}
