package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp (name = "TestServoPositions", group = "Competition")
//@Disabled
public class TestServoPositions extends OpMode {

    RobotTechnicolorRR robot;

    double position = 0;
    double positionChange = 0.1;

    GamepadEvents gamepad1;

    @Override
    public void init() {

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);
        robot.drive.setPoseEstimate(new Pose2d(0, 0, 0));
    
        Robot.writeToMatchFile( "Init Finished", true );
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
            robot.goalLift.claw.setPosition( position );
        if( gamepad1.right_trigger > 0.1 )
            robot.ringBlocker.setNumericalBlockerPosition( position );

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

    public void addLine() {
        telemetry.addLine();
    }

}
