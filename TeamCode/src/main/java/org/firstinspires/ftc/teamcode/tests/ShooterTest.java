package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robots.ShooterBot;

public class ShooterTest extends OpMode {

    ShooterBot robot;

    @Override
    public void init() {
        robot = new ShooterBot(hardwareMap, this);
    }

    @Override
    public void loop() {
        robot.ringShooter.launchRingVelocity(10, DistanceUnit.METER);
    }
}
