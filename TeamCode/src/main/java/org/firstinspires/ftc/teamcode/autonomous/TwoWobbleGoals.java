package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
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
import org.firstinspires.ftc.teamcode.utils.Vuforia;

@Autonomous(name = "Two Wobble Goals", group = "Competition")
public class TwoWobbleGoals extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private TensorFlowUtil.Stack stack;

    private final boolean pickUpRing = true;
    private final boolean experimental = false;
    private final boolean alternateQuadRoute = false;
    private final boolean scanRingsDuringInit = true;

    private MecanumVelocityConstraint speedVeloConstraint = new MecanumVelocityConstraint( 70, Math.toRadians( 80 ), 21 );
    private ProfileAccelerationConstraint speedAccConstraint = new ProfileAccelerationConstraint( 40 );

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

        telemetry.addLine( "Initialization Complete" );
        telemetry.update();

        Robot.writeToMatchFile( "Init Finished", true );

        new Thread( () -> {
            while( !isStarted() ) {
                if( isStopRequested() ) {
                    robot.tfod.deactivateTensorFlow();
                    if( Vuforia.getInstance().isRunning() )
                        Vuforia.getInstance().close();
                }
            }
        } ).start();

        if( scanRingsDuringInit )
            robot.tfod.runWhileNotStartedStackDetection();

        waitForStart();

        //Detect stack
        if( scanRingsDuringInit ) {
            //robot.tfod.stopWhileRecognition(); // should force runWhileStackDetection() to exit the while loop, count the stacks, and set the stack value
            stack = robot.tfod.getStack();
        } else
            detectStack();

        if( alternateQuadRoute && stack == Robot.STACK_QUAD ) {
            // move out of way of rings
            robot.drive( robot.trajectoryBuilder().lineToConstantHeading( new Vector2d( -45, -57 ) ).build() );

            // move wobble goal one to corner
            moveWobbleGoalOne();

            // drop off wobble goal 1
            dropWobbleGoal();

            // shoot the rings from lef tto right
            shootRings();

            // pick up the quad stack and shoot them in the high goal
            pickUpStackAndShoot();

        } else {

            // middle of stack and wobble goal is (-34.75, -30)

            //shoot// was 9.3
            robot.drive( robot.trajectoryBuilder()
                    .splineToConstantHeading( new Vector2d( -45, -52 ), 0 )
                    .splineToConstantHeading( new Vector2d( -36, -36 ), 90 )
                    .addDisplacementMarker( () -> {
                        robot.ringShooter.setFlyWheelMotorVelocity( 9.35, AngleUnit.RADIANS );
                    } )
                    .splineToConstantHeading( new Vector2d( -13, -4 ), 90 ).build() );
            shootRings();

            //if there is one ring in the stack, pick up the ring and shoot
            pickUpStackAndShoot();

            //Move wobble goal to correct zone
            moveWobbleGoalOne();

            //Drop wobble goal TODO: create method in robot class for this
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

        robot.goalLift.setGoalLiftPositionAsync( Robot.LIFT_LIFTED, 0.6, 800 );
        robot.drive.waitForIdle();


        //Return to center line

        while( opModeIsActive() && !isStopRequested() ) {
            idle();
        }
    }

    private void park() {

        if( stack == Robot.STACK_NONE )
            robot.driveAsync( robot.trajectoryBuilder().strafeRight( 8 ).splineToConstantHeading( new Vector2d( 10, -36 ), 0 ).build() );
        else
            robot.driveAsync( robot.trajectoryBuilder().lineTo( new Vector2d( 8, -36 ) ).build() );

    }

    private void moveWobbleGoalTwo() {

        if( stack == Robot.STACK_NONE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -15, -53, Math.toRadians( 180 ) ), 0, speedVeloConstraint, speedAccConstraint ).build() );
        } else if( stack == Robot.STACK_SINGLE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 10, -36, Math.toRadians( 180 ) ), 0, speedVeloConstraint, speedAccConstraint ).build() );
        } else {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 33, -53, Math.toRadians( 180 ) ), 0, speedVeloConstraint, speedAccConstraint ).build() );
        }
    }

    private void pickUpWobbleGoalTwo() {

        if( stack == Robot.STACK_NONE )
            robot.goalLift.setClawPosition( Robot.CLAW_CLOSED );
        if( stack == Robot.STACK_NONE || stack == Robot.STACK_QUAD )
            robot.goalLift.setGoalLiftPositionAsync( Robot.LIFT_LIFTED, 1.0, 800 );

        robot.driveAsync( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -16, -30, 0 ), 90, speedVeloConstraint, speedAccConstraint ).build() );
        robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LIFTED, 0.6, 700 );
        robot.drive.waitForIdle();

        robot.goalLift.setClawPositionAsync( Robot.CLAW_OPEN );
        robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LOWERED, 0.6, 500 );
        double radianSplice = stack == Robot.STACK_QUAD ? 10 : 6;
        robot.drive( robot.trajectoryBuilder().lineToLinearHeading( new Pose2d( -25.625 - 1.5, -30.5, Math.toRadians( 0/*radianSplice*/ ) ) ).build() );

        sleep( 300 );
        robot.goalLift.setClawPosition( Robot.CLAW_CLOSED );

        sleep( 1000 );
        robot.goalLift.setGoalLiftPositionAsync( Robot.LIFT_LIFTED, 1.0, 800 );
    }

    private void dropWobbleGoal() {

        robot.goalLift.setGoalLiftPosition( Robot.LIFT_LOWERED, 0.6, 700 );
        robot.goalLift.setClawPosition( Robot.CLAW_OPEN );
        sleep( 500 );
    }

    private void moveWobbleGoalOne() {

        if( stack == Robot.STACK_NONE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( -15, -60, Math.toRadians( 180 ) ), 90 ).build() );
        } else if( stack == Robot.STACK_SINGLE ) {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 20, -36, Math.toRadians( 180 ) ), 90 ).build() );
        } else {
            robot.drive( robot.trajectoryBuilder().splineToLinearHeading( new Pose2d( 33, -58, Math.toRadians( 180 ) ), 90 ).build() );
            //robot.setPosition(new Pose2d(33, -56));
            // off - puts wobble goal in center of square (4 rings)
        }
        robot.ringShooter.setIntakeMotorPower( 0 );
    }

    private void pickUpStackAndShoot() {
        /*telemetry.addLine( "Stack: " + stack );
        telemetry.addLine( "Other Stack: " + robot.tfod.getStack() );
        telemetry.update();*/
        if( (stack == Robot.STACK_SINGLE || stack == Robot.STACK_QUAD) && pickUpRing ) {

            double ringPosX = -37;

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
                long currentTime = System.currentTimeMillis();
                while (System.currentTimeMillis() < currentTime + 200) ;
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

            double initialTime = this.getRuntime();

            robot.ringShooter.setIntakeMotorPower( 1 );
            robot.ringShooter.setFlyWheelMotorVelocity( 11, AngleUnit.RADIANS );
            robot.drive( robot.trajectoryBuilder().lineToLinearHeading( new Pose2d( -7, ringPosX, 0 ) ).build() );

            if( stack == Robot.STACK_SINGLE )
                driveWaitShoot( -17, ringPosX, 10.075, 250 );
            else if( stack == Robot.STACK_QUAD ) {
                robot.drive( robot.trajectoryBuilder()
                        .lineToConstantHeading( new Vector2d( -34, ringPosX ),
                                new MecanumVelocityConstraint( 15, Math.toRadians( 60 ), 21 ),
                                new ProfileAccelerationConstraint( 40 ) )
                        .addDisplacementMarker( 10, () -> {
                            for( int i = 0; i <= 4; i++ ) {
                                long startTime = System.currentTimeMillis();
                                while( System.currentTimeMillis() < startTime + 300 ) ;
                                robot.ringShooter.pushRingTime();
                            }
                        } ).build() );
            }


            /*logAndPrint(this.getRuntime() - initialTime + " : shoot ring 1");
            driveWaitShoot( -17, ringPosX, 10.075, 250 );
    
            if (stack == Robot.STACK_QUAD) {
    
                robot.logAndPrint(this.getRuntime() - initialTime + " : shoot ring 2");
                driveWaitShoot( -21, ringPosX, 10.1, 0 );
                
                robot.logAndPrint(this.getRuntime() - initialTime + " : shoot ring 3");
                driveWaitShoot( -26, ringPosX, 10.3, 0 );
    
                robot.logAndPrint(this.getRuntime() - initialTime + " : shoot ring 4");
                driveWaitShoot( -31, ringPosX, 10.3, 0 );
        
                robot.ringShooter.launchRingAngularVelocity(10.3, false, 250);
            }*/
            robot.logAndPrint( this.getRuntime() - initialTime + " : set powers zero", true );
            robot.ringShooter.setFlyWheelMotorPower( 0 );

            /*
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

    public void driveWaitShoot( double posX, double posY, double omega, long speedUpTime ) {
        robot.driveAsync( robot.trajectoryBuilder().lineToConstantHeading( new Vector2d( posX, posY ) ).build() );
        robot.drive.waitForIdle();
        robot.ringShooter.launchRingAngularVelocity( omega, false, speedUpTime );
    }

    public void logAndPrint( String text ) {
        Robot.writeToMatchFile( text, false );
        telemetry.addLine( text );
        telemetry.update();
    }

    private void shootRings() {

        // middle of stack and wobble goal is (-34.75, -30)

        double shootPosX = -13;

        shootAndPrint( shootPosX, -4.875, FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false );

        shootAndPrint( shootPosX, -12.375, FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false );

        shootAndPrint( shootPosX, -19.875, FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true );
        
        /* old
        robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(shootPosX, -4.875, 0), 0).build());
        robot.ringShooter.setFlyWheelMotorVelocity(9.75, AngleUnit.RADIANS);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        robot.drive.waitForIdle();

        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();

        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(shootPosX, -12.375, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();

        robot.drive(robot.trajectoryBuilder().lineToLinearHeading(new Pose2d(shootPosX, -19.875, 0)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true, false);
        telemetry.addData("Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
         */
    }

    public void shootAndPrint( double posX, double posY, OpenGLMatrix target, boolean setPowerZero ) {
        robot.drive( robot.trajectoryBuilder().lineToLinearHeading( new Pose2d( posX, posY, 0 ) ).build() );
        robot.shootAtTarget( FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, setPowerZero, false );
        telemetry.addData( "Fly Wheel Speed", robot.ringShooter.rightFlyWheelMotor.getVelocity( AngleUnit.RADIANS ) );
        telemetry.update();
    }

    private void detectStack() {
        robot.drive( robot.trajectoryBuilder().lineToConstantHeading( new Vector2d( -56, -41.875 ) ).build() );
        robot.tfod.runStackDetection( 20000 );
        stack = robot.tfod.getStack();
    }
}
