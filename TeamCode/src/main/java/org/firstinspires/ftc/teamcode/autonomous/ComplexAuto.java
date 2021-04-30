package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil.Stack;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

import java.util.List;

@Autonomous(name = "Complex Auto", group = "Competition")
public class ComplexAuto extends LinearOpMode {

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

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR( hardwareMap, this );

        SoundLibrary.playStartup();

        robot.setPosition( new Pose2d( -61.125, -41.875 ) );
        robot.ringShooter.setPusherPosition( Robot.PUSHER_RETRACTED );
        robot.goalLift.setClawPosition( Robot.CLAW_CLOSED );
        robot.tfod.initTensorFlow();
        robot.tfod.setZoom( 2 );

        robot.logAndPrint( "Init Finished", true );

        new Thread( () -> {
            while( !isStopRequested() ) ;
            robot.tfod.deactivateTensorFlow();
            if( Vuforia.getInstance().isRunning() )
                Vuforia.getInstance().close();
        } ).start();

        //If we loose the expansion hub!
        normalLocalizer = robot.drive.getLocalizer();
        List<LynxModule> hubs = hardwareMap.getAll( LynxModule.class );
        new Thread( () -> {
            while( opModeIsActive() && !isStopRequested() ) {
                for( LynxModule hub : hubs ) {
                    if( hub.isNotResponding() && !hubWasNotResponding ) {
                        Pose2d currentPose = robot.drive.getPoseEstimate();
                        robot.drive.setLocalizer( new MecanumDrive.MecanumLocalizer( robot.drive ) );
                        robot.drive.setPoseEstimate( currentPose );
                        hubWasNotResponding = true;
                    } else if( hubWasNotResponding && !hub.isNotResponding() ) {
                        Pose2d currentPose = robot.drive.getPoseEstimate();
                        robot.drive.setLocalizer( normalLocalizer );
                        robot.drive.setPoseEstimate( currentPose );
                        hubWasNotResponding = false;
                    }
                }
            }
        } );

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

        while( opModeIsActive() && !isStopRequested() ) {
            idle();
        }
    }

    private void park() {

        if( stack == Robot.STACK_NONE )
            robot.driveAsync( robot.trajectoryBuilder().strafeRight( 8 ).splineToConstantHeading( new Vector2d( 14, -36 ), 0 ).build() );
        else
            robot.driveAsync( robot.trajectoryBuilder().lineTo( new Vector2d( 8, -36 ) ).build() );

    }

    private void moveWobbleGoalTwo() {

        if( stack == Robot.STACK_NONE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -15, -53, Math.toRadians( 180 ) ), 0 ).build() );
            dropWobbleGoal();
            robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LIFTED, 0.6, 500 );
        } else if( stack == Robot.STACK_SINGLE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 12, -32, Math.toRadians( 180 ) ), 0 ).build() );
            dropWobbleGoal();
        } else {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 33, -53, Math.toRadians( 180 ) ), 0 ).addTemporalMarker( 0.6, 0, this::dropWobbleGoalAsync ).build() );
        }
    }

    private void pickUpWobbleGoalTwo() {

        if( stack == Robot.STACK_NONE )
            robot.goalLift.setClawPosition( Robot.CLAW_CLOSED );
        if( stack == Robot.STACK_NONE || stack == Robot.STACK_QUAD )
            robot.goalLift.setGoalLiftPositionAsync( Robot.LIFT_LIFTED, 1.0, 800 );

        // robot's position to have the arm be perfectly centered on the wobble goal: ( -26.375 , -30.5 )
        // should drive to wobble goal position & lower the wobble goal arm
        robot.driveAsync( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -16, stack != Robot.STACK_NONE ? -30 : -30.5, 0 ), 90/*, speedVeloConstraint, speedAccConstraint*/ ).build() );
        robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LIFTED, 0.6, 700 );
        robot.drive.waitForIdle();

        // should drive back while closing the claw & raising the lift
        // close claw async, raise arm async, drive back
        robot.goalLift.setClawPositionAsync( Robot.CLAW_OPEN );
        robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LOWERED, 0.6, 500 );
        double radianSplice = stack == Robot.STACK_QUAD ? 10 : 6;
        robot.drive( robot.trajectoryBuilder().lineToLinearHeading( new Pose2d( -25.625 - (stack != Robot.STACK_SINGLE ? 1.5 : 2), stack != Robot.STACK_NONE ? -32 : -30.5, Math.toRadians( 0/*radianSplice*/ ) ) ).build() );

        sleep( 300 );
        robot.goalLift.setClawPosition( Robot.CLAW_CLOSED );

        sleep( 1000 );
        if( stack != Robot.STACK_SINGLE )
            robot.goalLift.setGoalLiftPositionAsync( Robot.LIFT_LIFTED, 1.0, 800 );
    }

    private void dropWobbleGoal() {

        robot.goalLift.setGoalLiftPosition( Robot.LIFT_LOWERED, 0.6, 700 );
        robot.goalLift.setClawPosition( Robot.CLAW_OPEN );
        robot.sleepRobot( 300 );
    }

    private void dropWobbleGoalAsync() {
        new Thread( () -> {
            robot.goalLift.setGoalLiftPosition( Robot.LIFT_LOWERED, 0.6, 700 );
            robot.goalLift.setClawPosition( Robot.CLAW_OPEN );
        } ).start();
    }

    private void moveWobbleGoalOne() {

        if( stack == Robot.STACK_NONE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -15, -60, Math.PI - 1e-6 ), 88 ).build() );
        } else if( stack == Robot.STACK_SINGLE ) {
            robot.drive( robot.trajectoryBuilder().splineToConstantHeading( new Vector2d( -24, -55 ), 270 ).build() );
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 20, -36, Math.PI - 1e-6 ), 88 ).build() );
        } else if( stack == Robot.STACK_QUAD ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 33, -60, Math.PI - 1e-6 ), 88 ).build() );
        }

        robot.ringShooter.setIntakeMotorPower( 0 );
    }

    private void pickUpStackAndShoot() {

        if( stack == Robot.STACK_SINGLE || stack == Robot.STACK_QUAD ) {

            double ringPosX = -36;

            robot.ringShooter.setIntakeMotorPower( 1 );
            robot.ringShooter.setFlyWheelMotorVelocity( 10, AngleUnit.RADIANS );
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -7, ringPosX, Math.toRadians( 0 ) ), 0 ).build() );

            if( stack == Robot.STACK_SINGLE )
                driveWaitShoot( -17, ringPosX, 10.1, 500 );
            else if( stack == Robot.STACK_QUAD ) {
                double[] velocities = { 10.4, 10.5, 10.8, 10.7 };
                robot.drive( robot.trajectoryBuilder().lineToConstantHeading( new Vector2d( -18, ringPosX ) ).build() );
                robot.drive( robot.trajectoryBuilder()
                        .lineToLinearHeading( new Pose2d( -36, ringPosX, 0 ),
                                new MecanumVelocityConstraint( 8, Math.toRadians( 60 ), 21 ),
                                new ProfileAccelerationConstraint( 40 ) )
                        .addDisplacementMarker( 0, () -> {
                            new Thread( () -> {
                                for( int i = 0; i < 4; i++ ) {
                                    robot.ringShooter.setFlyWheelMotorVelocity( velocities[i], AngleUnit.RADIANS );
                                    robot.sleepRobot( 300 );
                                    robot.ringShooter.pushRingTime();
                                }
                                robot.ringShooter.pushRingTime();
                                robot.ringShooter.pushRingTime();
                                robot.ringShooter.pushRingTime();
                                robot.ringShooter.pushRingTime();
                                robot.ringShooter.setFlyWheelMotorPower( 0 );
                            } ).start();
                        } ).build() );
            }
            robot.ringShooter.setIntakeMotorPower( 0 );

        }

    }

    public void driveWaitShoot( double posX, double posY, double omega, long speedUpTime ) {
        robot.driveAsync( robot.trajectoryBuilder().lineToConstantHeading( new Vector2d( posX, posY ) ).build() );
        robot.drive.waitForIdle();
        robot.ringShooter.launchRingAngularVelocity( omega, false, speedUpTime );
    }

    private void shootRings() {

        robot.drive( robot.trajectoryBuilder().addDisplacementMarker( () -> robot.ringShooter.setFlyWheelMotorVelocity( 9.4, AngleUnit.RADIANS ) ).splineToLinearHeading( new Pose2d( -4, -19, 0 ), 0 ).build() );
        robot.shootAtTarget( FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, false );
        robot.drive.turnTo( Math.toRadians( 5.5 ) );
        robot.shootAtTarget( FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false );
        robot.drive.turnTo( Math.toRadians( 11 ) );
        robot.shootAtTarget( FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false );
    }
}
