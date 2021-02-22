package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.ShooterBotRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;

@Autonomous
public class BasicShooterAuto extends LinearOpMode {

    private ShooterBotRR robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ShooterBotRR(hardwareMap);
        robot.setPosition(new Pose2d(-60, -48));

        waitForStart();

        robot.drive(robot.trajectoryBuilder().splineTo(new Vector2d(0, -18.5), 0).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT);

        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(0, -11)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT);

        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(0, -3.5)).build());
        robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT);

        robot.drive(robot.trajectoryBuilder().lineTo(new Vector2d(12, -3.5)).build());

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
