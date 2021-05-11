package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;

@Autonomous(group = "test")
public class ShootingTest extends LinearOpMode {

    RobotTechnicolorRR robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotTechnicolorRR(hardwareMap, this);
        robot.setPosition(new Pose2d(-61.125, -41.875));
        robot.ringShooter.setPusherPosition(Robot.PUSHER_RETRACTED);

        waitForStart();

        robot.ringShooter.setFlyWheelMotorVelocity(9.3, AngleUnit.RADIANS);

        robot.drive(robot.trajectoryBuilder().splineToLinearHeading(new Pose2d(-4, -19, 0), 0).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, false);
        robot.drive.turn(Math.toRadians(6.5));
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
        robot.drive.turn(Math.toRadians(6.5));
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);

    }
}
