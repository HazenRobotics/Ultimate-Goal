package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.ShooterBot;

@TeleOp(name = "Motor test", group="Test")
@Disabled
public class RingPusherTest extends OpMode {

    ShooterBot robot;

    @Override
    public void init() {

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        robot = new ShooterBot(hardwareMap, this);
    
        Robot.writeToMatchFile( "Init Finished", true );
    }

    @Override
    public void loop() {

        robot.ringShooter.pusher.setPosition(gamepad1.right_stick_x);
        telemetry.addData("Servo position", robot.ringShooter.pusher.getPosition());
        telemetry.update();

    }
}