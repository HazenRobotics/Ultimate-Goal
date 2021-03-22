package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp (name = "Technicolor", group = "Competition")
public class TeleOpTechnicolor extends OpMode {
    //TODO: Create button toggle for controllers

    RobotTechnicolorRR robot;

    final double LIFT_POWER = 0.5;
    final double SHOOTER_POWER = 0.85;
    final double INTAKE_POWER = 1.0;

    GamepadEvents gamepad1 = new GamepadEvents(super.gamepad1);

    @Override
    public void init() {
        robot = new RobotTechnicolorRR(hardwareMap, this);
    }

    @Override
    public void loop() {
        robot.drive.setWeightedDrivePower(new Pose2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        ));
        if(gamepad1.b.onPress())
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.CLOSED );
        if(gamepad1.x.onPress())
            robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN );

        if( gamepad1.y.onPress() )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LIFTED, LIFT_POWER, 1000 );
        if( gamepad1.a.onPress() )
            robot.goalLift.setGoalLiftPosition( GoalLift.LiftPosition.LOWERED, LIFT_POWER, 1000 );

        // ring shooter = gamepad1.right_trigger

        robot.ringShooter.setFlyWheelMotorPower( gamepad1.right_trigger*SHOOTER_POWER );

        // ring pusher (servo) = gamepad1.left_bumper
        if( gamepad1.left_bumper.onPress() )
            robot.ringShooter.pushRing();

        // intake = gamepad1.left_trigger
        if(gamepad1.right_bumper.onPress())
            robot.ringShooter.setIntakeMotorPower( robot.ringShooter.getIntakePower() > 0 ? 0 : INTAKE_POWER);
        //robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*INTAKE_POWER );

        robot.drive.update();
        telemetry.update();
        gamepad1.update();
    }
}
