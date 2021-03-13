package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.mechanisms.RingShooter;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotWood;

// TeleOp class for the new wooden robot
@TeleOp(name="TeleOpWood", group="teleop")
public class TeleOpWood extends OpMode {

    //Robot robot;
    RobotWood robot;

    public static boolean doTelemetry = true;

    @Override
    public void init() {
        robot = new RobotWood(hardwareMap, this);

        robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN );
        robot.ringShooter.setPusherPosition( RingShooter.PusherPosition.RETRACTED );

        telemetry.addLine("init finished");
        telemetry.update();

        Robot.writeToDefaultFile( "*******INIT FINISHED********", false, true );
    }

    @Override
    public void loop() {

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
                .addData("Intake", "Gp1: left trigger");
        addLine();

        // drive, strafe, rotate = gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x
        double drivePower = ( gamepad1.left_stick_button ? 1.0 : 0.4 );
        robot.mecanumDrive.drive( -gamepad1.left_stick_y*drivePower, gamepad1.left_stick_x*drivePower, gamepad1.right_stick_x*drivePower );

        // claw = gamepad1.b and gamepad1.x
        if(gamepad1.b)
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.CLOSED );
        if(gamepad1.x)
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN);

        // goal lift = gamepad1.y and gamepad1.a
        double liftPower = 0.5;
        if( gamepad1.y )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LIFTED, liftPower );
        if( gamepad1.a )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LOWERED, liftPower );


        // ring shooter = gamepad1.right_trigger
        double ringShooterPower = 0.75;
        robot.ringShooter.setFlyWheelMotorPower( gamepad1.right_trigger*ringShooterPower );

        // ring pusher (servo) = gamepad1.left_bumper
        if( gamepad1.left_bumper )
            robot.ringShooter.pushRing();

        // intake = gamepad1.left_trigger
        double intakePower = 0.75;
        robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*intakePower );

        addTelemetry();

        telemetry.update();

    }

    public void addTelemetry() {

        telemetry.addLine("left_stick_y  = " + gamepad1.left_stick_y);
        telemetry.addLine("left_stick_x  = " + gamepad1.left_stick_x);
        telemetry.addLine("right_stick_x = " + gamepad1.right_stick_x);
        addLine();

        telemetry.addLine("longitudinal position = " + robot.tracker.getLongitudinalPosition() + " (ticks), "
                + robot.mecanumDrive.convertTicksDist( robot.tracker.getLongitudinalPosition()) + " (in)" );
        telemetry.addLine("lateral position = " + robot.tracker.getLateralPosition() + " (ticks), "
                + robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition()) + " (in)" );
        addLine();

        telemetry.addLine( "Claw Position = " + robot.goalLift.getCurrentClawPosition() + " :: "  + robot.goalLift.getClawPosition() );
        telemetry.addLine( "Lift Position = " + robot.goalLift.getCurrentLiftPosition() + " :: " + robot.goalLift.getLiftPower() );
        addLine();

        telemetry.addLine( "Shooter Power = " + robot.ringShooter.getFlyWheelPower() );
        addLine();

        telemetry.addLine( "Pusher Position = " + robot.ringShooter.getPusherPosition() + " :: " + robot.ringShooter.getPusherLocation() );
        addLine();

        telemetry.addLine( "Intake Power = " + robot.ringShooter.getIntakePower() );
        addLine();

        telemetry.addLine("get360GyroHeading = " + robot.tracker.get360GyroHeading() );
        telemetry.addLine("getGyroHeading    = " + robot.tracker.getGyroHeading() );

    }

    public void addLine() {
        telemetry.addLine();
    }

}
