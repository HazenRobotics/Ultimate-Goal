package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp (name = "TestServoPositions", group = "Competition")
@Disabled
public class TestServoPositions extends OpMode {

    RobotTechnicolorRR robot;

    double position = 0;
    double positionChange = 0.1;

    GamepadEvents gamepad1;

    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);
        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
    }

    @Override
    public void loop() {

        if( gamepad1.dpad_up.onPress() ) {
            position += positionChange;
        } else if( gamepad1.dpad_down.onPress() ) {
            position -= positionChange;
        } else if( gamepad1.dpad_right.onPress() ) {
            positionChange += 0.01;
        } else if( gamepad1.dpad_left.onPress() ) {
            positionChange -= 0.01;
        }

        telemetry.addLine( "position: " + position );
        telemetry.addLine( "positionChange: " + positionChange );

        if( gamepad1.right_bumper.onPress() )
            robot.ringShooter.pusher.setPosition( position );
        if( gamepad1.left_bumper.onPress() )
            robot.goalLift.claw.setPosition( positionChange );

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
