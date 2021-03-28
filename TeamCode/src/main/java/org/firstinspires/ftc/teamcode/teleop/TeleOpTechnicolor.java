package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class TeleOpTechnicolor extends OpMode {

    RobotTechnicolorRR robot;

    final double LIFT_POWER = 0.5;
    final double SHOOTER_POWER = 0.85;
    final double INTAKE_POWER = 1.0;

    final double MAX_DRIVE_SPEED = 0.8;
    final double MIN_DRIVE_SPEED = 0.4;

    final double MAX_TURN_SPEED = 0.5;
    final double MIN_TURN_SPEED = 0.3;

    final long LIFT_TIME_LIMIT = 500;
    final long LOWER_TIME_LIMIT = 500;

    private double driveMult = MAX_DRIVE_SPEED;
    private double turnMult = MIN_TURN_SPEED;

    int negateIntake = 1;

    double velocity = 8;
    double velocityChange = 1;

    GamepadEvents gamepad1;
    Vuforia vuforia = Vuforia.getInstance();

    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);
        //robot.vuforiaLocalization.activateTracking();
    }

    @Override
    public void loop() {

        //addControlTelemtry();

        //Sprint control
        if(gamepad1.left_stick_button.onPress())
            driveMult = driveMult < MAX_DRIVE_SPEED ? MAX_DRIVE_SPEED : MIN_DRIVE_SPEED;
        if(gamepad1.right_stick_button.onPress())
            turnMult = turnMult < MAX_TURN_SPEED ? MAX_TURN_SPEED : MIN_TURN_SPEED;

        robot.teleopDrive(-gamepad1.left_stick_y*driveMult, gamepad1.left_stick_x*driveMult, -gamepad1.right_stick_x*turnMult);

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

        telemetry.addLine( "velocity: " + velocity );

        // claw position
        if(gamepad1.x.onPress())
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.CLOSED );
        if(gamepad1.b.onPress())
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN );

        // goal lift
        if( gamepad1.y.onPress() )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LIFTED, LIFT_POWER + 0.3, LIFT_TIME_LIMIT );
        if( gamepad1.a.onPress() )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LOWERED, LIFT_POWER, LOWER_TIME_LIMIT );

        // ring shooter = gamepad1.right_trigger
        robot.ringShooter.setFlyWheelMotorVelocity( gamepad1.right_trigger*velocity, AngleUnit.RADIANS );
        //robot.ringShooter.setFlyWheelMotorPower( gamepad1.right_trigger*SHOOTER_POWER );

        // ring pusher (servo) = gamepad1.left_bumper
        if( gamepad1.left_bumper.onPress() ) {
            robot.ringShooter.pushRing();
        }

        // intake = gamepad1.left_trigger
        if(gamepad1.right_bumper.onPress())
            robot.ringShooter.setIntakeMotorPower( robot.ringShooter.getIntakePower() > 0 ? 0 : INTAKE_POWER);
        //robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*INTAKE_POWER );

        //addMotorInfoTelemtry();

        if(gamepad1.start.onPress()) {
            robot.setPosition(FieldMap.toPose2d(robot.vuforiaLocalization.getRobotPosition(), robot.vuforiaLocalization.getRobotRotation()));
            robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-13, -25.5, 0), 0).build());
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, true);
            robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-13, -20)).build());
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
            robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(-13, -12.5)).build());
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);
        }

        addControlTelemtry();

        robot.drive.update();
        telemetry.update();
        gamepad1.update();
        robot.drive.update();
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
                .addData("Intake", "Gp1: left trigger");
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
