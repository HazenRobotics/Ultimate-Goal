package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.Vuforia;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalization;

import java.util.List;

@TeleOp (name = "Technicolor", group = "Competition")
public class TeleOpTechnicolor extends LinearOpMode {

    RobotTechnicolorRR robot;

    final double LIFT_POWER = 0.5;
    final double SHOOTER_POWER = 0.85;
    final double INTAKE_POWER = 1.0;

    final double MAX_DRIVE_SPEED = 0.8;
    final double MIN_DRIVE_SPEED = 0.4;

    final double MAX_TURN_SPEED = 0.35;
    final double MIN_TURN_SPEED = 0.2;

    final long LIFT_TIME_LIMIT = 500;
    final long LOWER_TIME_LIMIT = 500;

    private double driveMult = MIN_DRIVE_SPEED;
    private double turnMult = MIN_TURN_SPEED;

    int negateIntake = 1;

    double velocity = 9.25 ;
    double velocityChange = 1;
    double velocitySmallChange = 0.25;

    double maxVelocity = 10.5;
    double minVelocity = 9.25;

    private GamepadEvents gamepad1;
    private GamepadEvents gamepad2;
    private Vuforia vuforia = Vuforia.getInstance();
    private VuforiaLocalization vuforiaLocalizer;

    private final String VUFORIA_TRACKABLES_ASSET_NAME = "UltimateGoal";

    private Thread shootPowershotThread;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);
        gamepad2 = new GamepadEvents(super.gamepad2);
        if(!Vuforia.getInstance().isRunning()) {
            Vuforia.getInstance().start();
        }
        vuforiaLocalizer = new VuforiaLocalization(VUFORIA_TRACKABLES_ASSET_NAME);
        vuforiaLocalizer.activateTracking();

        shootPowershotThread = new Thread(() -> {
            robot.setPosition(new Pose2d(63, 15, 0)); // 61, 14 if the robot is 18" wide: 70.125-robotwidth/2, 23.125-robotwidth/2
            robot.driveAsync(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-13, -7.5, 0), 0).build());
            robot.ringShooter.setFlyWheelMotorVelocity(8, AngleUnit.RADIANS);
            robot.drive.waitForIdle();
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, false, false);
            robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-13, -15)).build());
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
            robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-13, -22.5)).build());
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, true, false);
        });

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            //addControlTelemtry();

            //Sprint control
        /*
        if(gamepad1.left_stick_button.onPress())
            driveMult = driveMult < MAX_DRIVE_SPEED ? MAX_DRIVE_SPEED : MIN_DRIVE_SPEED;
        if(gamepad1.right_stick_button.onPress())
            turnMult = turnMult < MAX_TURN_SPEED ? MAX_TURN_SPEED : MIN_TURN_SPEED;
         */
            driveMult = gamepad1.left_stick_button.getValue() ? MAX_DRIVE_SPEED : MIN_DRIVE_SPEED;
            turnMult = gamepad1.right_stick_button.getValue() ? MAX_TURN_SPEED : MIN_TURN_SPEED;

            robot.teleopDrive(-gamepad1.left_stick_y*driveMult, gamepad1.left_stick_x*driveMult, -gamepad1.right_stick_x*turnMult);

            if( gamepad2.y.onPress() )
                velocity = maxVelocity;
            else if( gamepad2.a.onPress() )
                velocity = minVelocity;

        /*
        //D-pad rotation control
        if(gamepad1.dpad_up.onPress()) {
            robot.drive.turnToAsync(0);
        } else if(gamepad1.dpad_down.onPress()) {
            robot.drive.turnToAsync(Math.toRadians(180));
        } else if(gamepad1.dpad_left.onPress()) {
            robot.drive.turnToAsync(Math.toRadians(270));
        } else if(gamepad1.dpad_right.onPress()) {
            robot.drive.turnToAsync(Math.toRadians(90));
        }
        */

            // negate the intake direction on left trigger press
            //if( gamepad1.left_trigger.onPress() ) negateIntake *= -1;

            // increases and decreases the velocity of the flyWheels
            if( gamepad1.dpad_up.onPress() )
                velocity += velocityChange;
            else if( gamepad1.dpad_down.onPress() )
                velocity -= velocityChange;
            else if( gamepad1.dpad_right.onPress() )
                velocity += velocitySmallChange;
            else if( gamepad1.dpad_left.onPress() )
                velocity -= velocitySmallChange;

            telemetry.addLine( "velocity: " + velocity );

            // claw position
            if(gamepad1.x.onPress())
                robot.goalLift.setClawPosition( GoalLift.ClawPosition.CLOSED );
            if(gamepad1.b.onPress())
                robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN );

            // goal lift
            if( gamepad1.y.onPress() )
                robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LIFTED, LIFT_POWER + 0.3, LIFT_TIME_LIMIT );
            if( gamepad1.a.onPress() )
                robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LOWERED, LIFT_POWER, LOWER_TIME_LIMIT );

            // ring shooter = gamepad1.right_trigger
            robot.ringShooter.setFlyWheelMotorVelocity( gamepad1.right_trigger*velocity, AngleUnit.RADIANS );
            //robot.ringShooter.setFlyWheelMotorPower( gamepad1.right_trigger*SHOOTER_POWER );

            // ring pusher (servo) = gamepad1.left_bumper
            if( gamepad1.left_bumper.onPress() ) {
                robot.ringShooter.pushRing();
            }

            // intake = gamepad1.left_trigger
            if(gamepad1.right_bumper.onPress() || gamepad2.right_bumper.onPress())
                robot.ringShooter.setIntakeMotorPower( robot.ringShooter.getIntakePower() > 0 ? 0 : INTAKE_POWER);
            else if(gamepad1.left_trigger > 0.2 || gamepad2.left_trigger > 0.2)
                robot.ringShooter.setIntakeMotorPower(-INTAKE_POWER);
            //robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*INTAKE_POWER );

            //addMotorInfoTelemtry();

            if(gamepad1.back.onPress()) {
                if(shootPowershotThread.isAlive()) {
                    shootPowershotThread.interrupt();
                }
                else {
                    shootPowershotThread.start();
                }

            }

            addControlTelemtry();

            vuforiaLocalizer.updateRobotLocation();
            telemetry.update();
            gamepad1.update();
            gamepad2.update();
            robot.drive.update();

            if(isStopRequested()) {
                if(vuforia.isRunning()) {
                    vuforia.close();
                }
            }
        }
    }


    public void addControlTelemtry() {

        telemetry.addLine("            Controls:");
        telemetry.addData("Drive ", "Gp1: left stick y (axis)")
                .addData("Strafe", "Gp1: left stick x (axis)")
                .addData("Rotate", "Gp1: right stick x (axis)")
                .addData("Open Claw ", "Gp1: b")
                .addData("Close Claw", "Gp1: x")
                .addData("Lift Goal Lift ", "Gp1: y")
                .addData("Lower Goal Lift", "Gp1: a")
                .addData("Ring Shooter", "Gp1: right trigger")
                .addData("Ring Pusher ", "Gp1: left bumper")
                .addData("Intake Toggle", "Gp1: right trigger")
                .addData("Intake", "Gp1: left trigger")
                .addData("Lifted Button", robot.goalLift.liftedButtonPressed())
                .addData("Lowered Button", robot.goalLift.loweredButtonPressed())
                .addData("Position Estimate", robot.drive.getPoseEstimate());
        addLine();
    }

    public void addInfoTelemetry() {

        telemetry.addLine("left_stick_y  = " + gamepad1.left_stick_y );
        telemetry.addLine("left_stick_x  = " + gamepad1.left_stick_x );
        telemetry.addLine("right_stick_x = " + gamepad1.right_stick_x );
        addLine();

        //telemetry.addLine("longitudinal position = " + robot.tracker.getLongitudinalPosition() + " (ticks), " + robot.mecanumDrive.convertTicksDist( robot.tracker.getLongitudinalPosition()) + " (in)" );
        //telemetry.addLine("lateral position = " + robot.tracker.getLateralPosition() + " (ticks), " + robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition()) + " (in)" );
        addLine();

        telemetry.addLine( "Claw Position = " + robot.goalLift.getClawLocation() + " :: "  + robot.goalLift.getClawPosition() );
        telemetry.addLine( "Lift Position = " + robot.goalLift.getCurrentLiftPosition() + " :: " + robot.goalLift.getLiftPower() );
        addLine();

        telemetry.addLine( "Shooter Power = " + robot.ringShooter.getFlyWheelPower() );
        addLine();

        telemetry.addLine( "Pusher Position = " + robot.ringShooter.getPusherLocation() + " :: " + robot.ringShooter.getPusherPosition() );
        addLine();

        telemetry.addLine( "Intake Power = " + robot.ringShooter.getIntakePower() );
        addLine();

        //telemetry.addLine("getGyroHeading    = " + robot.tracker.getGyroHeading() );
        //telemetry.addLine("get360GyroHeading = " + robot.tracker.get360GyroHeading() );

    }

    public void addLine() {
        telemetry.addLine();
    }

    public void addMotorInfoTelemtry() {

        //(leftFront, leftRear, rightRear, rightFront)

        List<Double> positions = robot.drive.getWheelPositions();

        telemetry.addData( "leftFront", "Pos 0: " + positions.get( 0 ) );
        telemetry.addData( "leftRear", "Pos 1: " + positions.get( 1 ) );
        telemetry.addData( "rightRear", "Pos 2: " + positions.get( 2 ) );
        telemetry.addData( "rightFront", "Pos 3: " + positions.get( 3 ) );

        List<Double> velocities = robot.drive.getWheelVelocities();

        addLine();

        telemetry.addData( "leftFront", "Vel 0: " + velocities.get( 0 ) );
        telemetry.addData( "leftRear", "Vel 1: " + velocities.get( 1 ) );
        telemetry.addData( "rightRear", "Vel 2: " + velocities.get( 2 ) );
        telemetry.addData( "rightFront", "Vel 3: " + velocities.get( 3 ) );

        addLine();

    }

}
