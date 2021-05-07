package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drives.RRTwoWheelTrackingLocalizer;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Two Wobble Goals", group = "Competition")
public class TwoWobbleGoals extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private TensorFlowUtil.Stack stack;

    private final boolean experimental = false;
    private final boolean alternateQuadRoute = false;

    /*private MecanumVelocityConstraint speedVeloConstraint = new MecanumVelocityConstraint(65, Math.toRadians(80), 21);
    private ProfileAccelerationConstraint speedAccConstraint = new ProfileAccelerationConstraint(40);*/

    private boolean hubWasNotResponding = false;
    private Localizer normalLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createMatchLogFile(this.getClass().getSimpleName());

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playRandomStartup();

        robot.setPosition(new Pose2d(-61.125, -41.875));
        robot.ringShooter.setPusherPosition(Robot.PUSHER_RETRACTED);
        robot.goalLift.setClawPosition(Robot.CLAW_CLOSED);
        robot.tfod.initTensorFlow();
        robot.tfod.setZoom(2);

        robot.logAndPrint("Init Finished", true);

        new Thread(() -> {
            while (!isStopRequested()) ;
            robot.tfod.deactivateTensorFlow();
            if (Vuforia.getInstance().isRunning())
                Vuforia.getInstance().close();
        }).start();

        //If we loose the expansion hub!
        normalLocalizer = robot.drive.getLocalizer();
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        new Thread(() -> {
            while (opModeIsActive() && !isStopRequested()) {
                for (LynxModule hub : hubs) {
                    if (hub.isNotResponding() && !hubWasNotResponding) {
                        Pose2d currentPose = robot.drive.getPoseEstimate();
                        robot.drive.setLocalizer(new MecanumDrive.MecanumLocalizer(robot.drive));
                        robot.drive.setPoseEstimate(currentPose);
                        hubWasNotResponding = true;
                        SoundLibrary.playAudio("nooo");
                    } else if (hubWasNotResponding && !hub.isNotResponding()) {
                        Pose2d currentPose = robot.drive.getPoseEstimate();
                        robot.drive.setLocalizer(normalLocalizer);
                        robot.drive.setPoseEstimate(currentPose);
                        hubWasNotResponding = false;
                    }
                }
            }
        }).start();

        robot.tfod.runWhileNotStartedStackDetectionSpeed();

        waitForStart();

        //Detect stack
        stack = robot.tfod.getStack();

        // middle of the stack and wobble goal is (-34.75, -30)

        //shoot// was 9.3
        /*cool path
        robot.drive(robot.trajectoryBuilder()
                .splineToConstantHeading(new Vector2d(-46, -52), 0)
                .splineToConstantHeading(new Vector2d(-36, -36), 90)
                .addDisplacementMarker(() -> robot.ringShooter.setFlyWheelMotorVelocity(9.35, AngleUnit.RADIANS))
                .splineToConstantHeading(new Vector2d(-13, -4), 90).build());
        */
        //robot.drive(robot.trajectoryBuilder().splineToConstantHeading(new Vector2d(-12, -50), 90).addDisplacementMarker(() -> robot.ringShooter.setFlyWheelMotorVelocity(9.35, AngleUnit.RADIANS)).splineToConstantHeading(new Vector2d(-10, 0), 0).build());

        //Move wobble goal to correct zone
        moveWobbleGoalOne();

        dropWobbleGoal();

        //robot.drive(robot.trajectoryBuilder().addDisplacementMarker(() -> robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.8, 500)).splineToLinearHeading(new Pose2d(-4, -4, 0), 90).addTemporalMarker(0.5, 0, () -> robot.ringShooter.setFlyWheelMotorVelocity(9.45, AngleUnit.RADIANS)).build());

        shootRings();
        //if there is one ring in the stack, pick up the ring and shoot
        pickUpStackAndShoot();


        //Pick up 2nd wobble goal
        pickUpWobbleGoalTwo();

        //Move wobble goal to correct zone (slightly to the left or back if single)
        moveWobbleGoalTwo();

        //park
        park();

        //robot.goalLift.setGoalLiftPositionAsync(Robot.LIFT_LIFTED, 0.6, 800);
        robot.drive.waitForIdle();


        //Return to center line

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    private void park() {

        if (stack == Robot.STACK_NONE)
            robot.driveAsync(robot.trajectoryBuilder().strafeRight(8).splineToConstantHeading(new Vector2d(14, -36), 0).build());
        else
            robot.driveAsync(robot.trajectoryBuilder().lineTo(new Vector2d(8, -36)).build());

    }

    private void moveWobbleGoalTwo() {

        if (stack == Robot.STACK_NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-15, -51, Math.toRadians(180)), 0).build());
            dropWobbleGoal();
            robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 500);
        } else if (stack == Robot.STACK_SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(12, -32, Math.toRadians(180)), 0).build());
            dropWobbleGoal();
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(33, -51, Math.toRadians(180)), 0).addTemporalMarker(0.6, 0, () -> robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LOWERED, 0.8, 800)).build());
            dropWobbleGoal();
            // no dropWobbleGoal() here?
        }
    }

    private void pickUpWobbleGoalTwo() {

        /*if (stack == Robot.STACK_NONE)
            robot.goalLift.setClawPosition(Robot.CLAW_CLOSED);
        if (stack == Robot.STACK_NONE || stack == Robot.STACK_QUAD)
            robot.goalLift.setGoalLiftPositionAsync(Robot.LIFT_LIFTED, 1.0, 800);*/

        // robot's position to have the arm be perfectly centered on the wobble goal: ( -26.375 , -30.5 )
        // should drive to wobble goal position & lower the wobble goal arm
        robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-16, stack != Robot.STACK_NONE ? -30 : -30.5, 0), 90/*, speedVeloConstraint, speedAccConstraint*/).build());
        //robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 700);
        robot.drive.waitForIdle();

        // should drive back while closing the claw & raising the lift
        // close claw async, raise arm async, drive back
        robot.goalLift.setClawPositionAsync(Robot.CLAW_OPEN);
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LOWERED, 0.6, 500);
        //double radianSplice = stack == Robot.STACK_QUAD ? 10 : 6;
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(-25.625 - (1.5), stack != Robot.STACK_NONE ? -32 : -30.5, Math.toRadians(0/*radianSplice*/))).build());

        sleep(300);
        robot.goalLift.setClawPosition(Robot.CLAW_CLOSED);

        sleep(1000);
        if(stack != Robot.STACK_SINGLE)
            robot.goalLift.setGoalLiftPositionAsync(Robot.LIFT_LIFTED, 1.0, 800);
    }

    private void dropWobbleGoal() {

        SoundLibrary.playRandomSmash();

        robot.goalLift.setGoalLiftPosition(Robot.LIFT_LOWERED, 0.6, 700);
        robot.goalLift.setClawPosition(Robot.CLAW_OPEN);
        robot.sleepRobot(300);
    }

    private void dropWobbleGoalAsync() {
        new Thread(() -> {
            robot.goalLift.setGoalLiftPosition(Robot.LIFT_LOWERED, 0.6, 700);
            robot.goalLift.setClawPosition(Robot.CLAW_OPEN);
        }).start();
    }

    private Pose2d newPos2d180(double x, double y) {
        return new Pose2d(x, y, Math.PI - 1e-6);
    }

    private void moveWobbleGoalOne() {

        Pose2d position = stack == Robot.STACK_QUAD ? newPos2d180(33, -60) : (stack == Robot.STACK_SINGLE ? newPos2d180(20, -36) : newPos2d180(-15, -60));

        if (stack == Robot.STACK_SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToConstantHeading(new Vector2d(-24, -55), 270).build());
        }
        robot.drive(robot.trajectoryBuilder().splineToLinearHeading(position, 88).build());

        /*
        if (stack == Robot.STACK_NONE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-15, -60, Math.toRadians(180)), 90).build());
        } else if (stack == Robot.STACK_SINGLE) {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(20, -36, Math.toRadians(180)), 90).build());
        } else {
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(33, -58, Math.toRadians(180)), 90).build());
            //robot.setPosition(new Pose2d(33, -56));
            // off - puts wobble goal in center of square (4 rings)
        }
        */

        robot.ringShooter.setIntakeMotorPower(0);
    }

    private void pickUpStackAndShoot() {

        if (stack == Robot.STACK_SINGLE || stack == Robot.STACK_QUAD) {

            double ringPosX = -36;

            /* see you later

            robot.ringShooter.setIntakeMotorPower(1);
            robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-10, ringPosX)).build());
            robot.drive.waitForIdle();

            if (!experimental) {

                robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-24, ringPosX)).build());
                sleep(500);
                robot.ringShooter.setIntakeMotorPower(0);
                if (stack == robot.STACK_QUAD) {
                    robot.ringShooter.launchRingAngularVelocity(10, false, false);
                    robot.ringShooter.launchRingAngularVelocity(10, false, false);
                }
                robot.ringShooter.launchRingAngularVelocity(10, true, false);

            } else {

                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-30, ringPosX)).build());
                robot.sleepRobot(300);
                //long currentTime = System.currentTimeMillis();
                //while (System.currentTimeMillis() < currentTime + 200) ;
                if (stack == robot.STACK_QUAD) {
                    robot.ringShooter.launchRingAngularVelocity(10, false, false);
                    robot.ringShooter.launchRingAngularVelocity(10.1, false, false);
                    robot.ringShooter.launchRingAngularVelocity(10.2, false, false);
                }
                robot.ringShooter.launchRingAngularVelocity(10.2, true, false);
                robot.drive.waitForIdle();
                robot.ringShooter.setIntakeMotorPower(0);
            }
             */

            robot.ringShooter.setIntakeMotorPower(1);
            robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-10, ringPosX, Math.toRadians(0)), 270).build());

            if (stack == Robot.STACK_SINGLE) {
                driveWaitShoot(-17, ringPosX, 10.0, 500);
                robot.ringShooter.setFlyWheelMotorPower(0);
                robot.ringShooter.setIntakeMotorPower(0);
            }
            else if (stack == Robot.STACK_QUAD) {
                double[] velocities = {10.2, 10.3, 10.6, 10.2};
                robot.drive(robot.trajectoryBuilder()
                        .lineToLinearHeading(new Pose2d(-34, ringPosX, 0),
                                new MecanumVelocityConstraint(8, Math.toRadians(60), 21),
                                new ProfileAccelerationConstraint(40))
                        .addDisplacementMarker(4, () -> {
                            new Thread(() -> { for (int i = 0; i < 4; i++) {
                                robot.ringShooter.setFlyWheelMotorVelocity(velocities[i], AngleUnit.RADIANS);
                                robot.sleepRobot(400);
                                robot.ringShooter.pushRingTime();
                            }
                            robot.ringShooter.pushRingTime();
                            robot.sleepRobot(400);
                            robot.ringShooter.pushRingTime();
                            robot.ringShooter.setIntakeMotorPower(0);
                            robot.ringShooter.setFlyWheelMotorPower(0);}).start();
                        }).build());
            }

            /*logAndPrint(this.getRuntime() - initialTime + " : shoot ring 1");
            driveWaitShoot( -17, ringPosX, 10.075, 250 );
            robot.ringShooter.setFlyWheelMotorPower(0);

            if (stack == Robot.STACK_QUAD) {

                robot.logAndPrint(this.getRuntime() - initialTime + " : shoot ring 2");
                driveWaitShoot( -21, ringPosX, 10.1, 0 );

                robot.logAndPrint(this.getRuntime() - initialTime + " : shoot ring 3");
                driveWaitShoot( -26, ringPosX, 10.3, 0 );

                robot.logAndPrint(this.getRuntime() - initialTime + " : shoot ring 4");
                driveWaitShoot( -31, ringPosX, 10.3, 0 );

                robot.ringShooter.launchRingAngularVelocity(10.3, false, 250);
            }

            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-17, ringPosX)).build()); // pick up ring 1
            robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 1");
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.075, false, 250);

            if (stack == robot.STACK_QUAD) {
                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-21, ringPosX)).build()); // pick up ring 2
                robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 2");
                robot.drive.waitForIdle();
                robot.ringShooter.launchRingAngularVelocity(10.1, false, 0);

                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-26, ringPosX)).build()); // pick up ring 3
                robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 3");
                robot.drive.waitForIdle();
                robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);

                robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-31, ringPosX)).build()); // pick up ring 4
                robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 4");
                robot.drive.waitForIdle();
                robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);

                robot.ringShooter.launchRingAngularVelocity(10.3, false, 250);
            }
             */
        }

    }

    public void driveWaitShoot(double posX, double posY, double omega, long speedUpTime) {
        robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(posX, posY)).build());
        robot.drive.waitForIdle();
        robot.ringShooter.launchRingAngularVelocity(omega, false, speedUpTime);
    }

    private void shootRings() {
        robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LIFTED, 0.6, 800);

        /*if (true stack == Robot.STACK_NONE) {
            shootAndPrint(-4, -6, FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false);

            shootAndPrint(-4, -13, FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false);

            shootAndPrint(-4, -21, FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true);
        }*/

        /*shootAndPrint(shootPosX, -4, FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false);

        shootAndPrint(shootPosX, -4, FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false);

        shootAndPrint(shootPosX, -4, FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true);*/

        robot.drive(robot.trajectoryBuilder().addDisplacementMarker(() -> robot.ringShooter.setFlyWheelMotorVelocity(9.1, AngleUnit.RADIANS)).splineToLinearHeading(new Pose2d(-4, -20.5, 0), 0).build());
        robot.drive.turn(Math.toRadians(0));
        telemetry.addData("Angle", robot.drive.getExternalHeading());
        telemetry.addData("Flywheel Velocity", robot.ringShooter.getFlyWheelVelocity(false));
        telemetry.update();
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, false);
        robot.drive.turn(Math.toRadians(6));
        telemetry.addData("Angle", robot.drive.getExternalHeading());
        telemetry.addData("Flywheel Velocity", robot.ringShooter.getFlyWheelVelocity(false));
        telemetry.update();
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
        robot.drive.turn(Math.toRadians(6));
        telemetry.addData("Angle", robot.drive.getExternalHeading());
        telemetry.addData("Flywheel Velocity", robot.ringShooter.getFlyWheelVelocity(false));
        telemetry.update();
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);
    }

    public void shootAndPrint(double posX, double posY, OpenGLMatrix target, boolean setPowerZero) {
        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(posX, posY, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, setPowerZero, false);
        robot.logAndPrint("Fly Wheel Speed :: " + robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS), true);

    }
}
