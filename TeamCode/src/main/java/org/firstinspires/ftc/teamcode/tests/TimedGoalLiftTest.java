package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="TimedGoalLiftTest", group="Test")
//@Disabled
public class TimedGoalLiftTest extends OpMode {
    RobotTechnicolorRR robot;

    TouchSensor newLiftedButton;
    TouchSensor newLoweredButton;

    GamepadEvents gamepad1;

    final double LIFT_POWER = 0.5;

    long timeToLift = 0;

    final long LIFT_TIME_LIMIT = 500;

    @Override
    public void init() {

        Robot.createDefaultMatchLogFileName( this.getClass().getName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);

        newLiftedButton = hardwareMap.touchSensor.get("liftedButton");
        newLoweredButton = hardwareMap.touchSensor.get("loweredButton");
    }

    @Override
    public void loop() {

        // goal lift
        if( gamepad1.y.onPress() ) {
            timeToLift = System.currentTimeMillis();
            robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LIFTED, LIFT_POWER + 0.3, LIFT_TIME_LIMIT);
            timeToLift = System.currentTimeMillis() - timeToLift;
        }
        if( gamepad1.a.onPress() ) {
            timeToLift = System.currentTimeMillis();
            robot.goalLift.setGoalLiftPosition(GoalLift.LiftPosition.LOWERED, LIFT_POWER, LIFT_TIME_LIMIT);
            timeToLift = System.currentTimeMillis() - timeToLift;
        }

        telemetry.addLine( "time: " + timeToLift );

        telemetry.addLine( "liftedButton: " + robot.goalLift.liftedButtonPressed() );
        telemetry.addLine( "loweredButton: " + robot.goalLift.loweredButtonPressed() );

        telemetry.addLine( "newLiftedButton: " + newLiftedButton.isPressed() );
        telemetry.addLine( "newLoweredButton: " + newLoweredButton.isPressed() );

        telemetry.update();
        gamepad1.update();

    }
}
