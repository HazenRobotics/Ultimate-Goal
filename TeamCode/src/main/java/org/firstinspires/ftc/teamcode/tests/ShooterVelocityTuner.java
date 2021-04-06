package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "Shooter Velocity Tuner", group = "Tuner")
//@Disabled
public class ShooterVelocityTuner extends OpMode {
    RobotTechnicolorRR robot;
    double velocity = 8;
    GamepadEvents gamepad1;

    @Override
    public void init() {

        Robot.createDefaultMatchLogFileName( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);
        gamepad1 = new GamepadEvents(super.gamepad1);
    
        Robot.writeToMatchDefaultFile( "Init Finished", true );
    }

    @Override
    public void loop() {

        robot.ringShooter.setFlyWheelMotorVelocity(velocity, AngleUnit.RADIANS);

        if(gamepad1.dpad_up.onPress()) {
            velocity += 1;
        }
        if(gamepad1.dpad_down.onPress()) {
            velocity -= 1;
        }
        telemetry.addData("Target Velocity", velocity);

        telemetry.addData("Left Flywheel Velocity", robot.ringShooter.leftFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("Right Flywheel Velocity", robot.ringShooter.rightFlyWheelMotor.getVelocity(AngleUnit.RADIANS));
        telemetry.update();
        gamepad1.update();
    }
}
