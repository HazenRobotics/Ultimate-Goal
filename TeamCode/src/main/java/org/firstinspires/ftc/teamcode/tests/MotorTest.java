package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.ShooterBot;

@TeleOp(name = "Motor test")
@Disabled
public class MotorTest extends OpMode {

    ShooterBot robot;


    @Override
    public void init() {
        robot = new ShooterBot(hardwareMap, this);
    }

    @Override
    public void loop() {

        robot.ringShooter.setIntakeMotorPower(0.5);

    }
}