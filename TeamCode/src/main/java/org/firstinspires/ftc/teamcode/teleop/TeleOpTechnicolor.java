package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp (name = "Technicolor", group = "Competition")
public class TeleOpTechnicolor extends OpMode {

    RobotTechnicolorRR robot;

    final double LIFT_POWER = 0.5;
    final double SHOOTER_POWER = 0.85;
    final double INTAKE_POWER = 1.0;

    double velocity = 50;
    double velocityChange = 50;
    double velocityChangeChange = 10;

    GamepadEvents gamepad1;

    private double sprintMult = 0.5;

    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);
        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {
        robot.drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y * sprintMult,
                -gamepad1.left_stick_x * sprintMult,
                -gamepad1.right_stick_x * sprintMult
        ));

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

        // increases and decreases the velocity of the flyWheels
        if( gamepad1.dpad_up.onPress() ) {
            velocity += velocityChange;
        } else if( gamepad1.dpad_down.onPress() ) {
            velocity -= velocityChange;
        } else if( gamepad1.dpad_right.onPress() ) {
            velocityChange += velocityChangeChange;
        } else if( gamepad1.dpad_left.onPress() ) {
            velocityChange -= velocityChangeChange;
        }

        telemetry.addLine( "velocity: " + velocity );
        telemetry.addLine( "velocityIncrease: " + velocityChange);

        //Sprint control
        if(gamepad1.left_stick_button.onPress()) {
            sprintMult = sprintMult < 1 ? 1 : 0.5;
        }
        if(gamepad1.b.onPress())
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.CLOSED );
        if(gamepad1.x.onPress())
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN );

        if( gamepad1.y.onPress() )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LIFTED, LIFT_POWER + 0.3, 1000 );
        if( gamepad1.a.onPress() )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LOWERED, LIFT_POWER, 1000 );

        // ring shooter = gamepad1.right_trigger
        robot.ringShooter.launchRingAngularVelocity( gamepad1.right_trigger*velocity, false );

        //robot.ringShooter.setFlyWheelMotorPower( gamepad1.right_trigger*SHOOTER_POWER );

        // ring pusher (servo) = gamepad1.left_bumper
        if( gamepad1.left_bumper.onPress() ) {
            robot.ringShooter.pushRing();
        }

        // intake = gamepad1.left_trigger
        if(gamepad1.right_bumper.onPress())
            robot.ringShooter.setIntakeMotorPower( robot.ringShooter.getIntakePower() > 0 ? 0 : INTAKE_POWER);
        //robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*INTAKE_POWER );

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
                .addData("Open Claw ", "Gp1: x")
                .addData("Close Claw", "Gp1: b")
                .addData("Lift Goal Lift ", "Gp1: y")
                .addData("Lower Goal Lift", "Gp1: x")
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

}
