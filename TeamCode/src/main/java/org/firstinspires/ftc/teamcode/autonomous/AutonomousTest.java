package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.TensorFlowUtil;
import org.firstinspires.ftc.teamcode.utils.Vuforia;

// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.
// This example assumes there is one encoder, attached to the left motor

@Autonomous(name = "AutonomousTest", group = "autonomous")
//@disabled
public class AutonomousTest extends LinearOpMode {

    private RobotTechnicolorRR robot;
    private TensorFlowUtil.Stack stack;

    double initialTime = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createMatchLogFile(this.getClass().getSimpleName());

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playStartup();

        //==========================================================================================

        //testTensorFlowInit();

        // testQuadShooting();

    }

    private void testTensorFlowInit() {

        robot.setPosition(new Pose2d(-61.125, -41.875));
        robot.tfod.initTensorFlow();
        robot.tfod.setZoom(2);

        robot.logAndPrint("Init Finished", true);

        new Thread(() -> {
            while (!isStarted()) {
                if (isStopRequested()) {
                    robot.tfod.deactivateTensorFlow();
                    if (Vuforia.getInstance().isRunning())
                        Vuforia.getInstance().close();
                }
            }
        }).start();

        robot.tfod.runWhileNotStartedStackDetectionSpeed();

        waitForStart();

        //Detect stack
        stack = robot.tfod.getStack();

        robot.logAndPrint(" Stack :: " + stack);
    }

    private void testQuadShooting() {

        robot.setPosition(new Pose2d(9.375, -39.125));

        robot.logAndPrint("Init Finished", true);

        waitForStart();

        double ringPosX = -35.375;

        initialTime = this.getRuntime();

        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(9.375, ringPosX)).build());

        stack = TensorFlowUtil.Stack.QUAD;

        robot.ringShooter.setIntakeMotorPower(1);
        robot.ringShooter.setFlyWheelMotorVelocity(10, AngleUnit.RADIANS);
        robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-7, ringPosX)).build());
        robot.drive.waitForIdle();

        robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-17, ringPosX)).build()); // pick up ring 1
        robot.drive.waitForIdle();
        robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 1");
        robot.ringShooter.launchRingAngularVelocity(10.075, false, 250);
        if (stack == TensorFlowUtil.Stack.QUAD) {
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-21, ringPosX)).build()); // pick up ring 2
            robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 2");
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.1, false, 0);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-26, ringPosX)).build()); // pick up ring 3
            robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 3");
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);
            robot.driveAsync(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-30, ringPosX)).build()); // pick up ring 4
            robot.logAndPrint(this.getRuntime() - initialTime + " shoot ring 4");
            robot.drive.waitForIdle();
            robot.ringShooter.launchRingAngularVelocity(10.3, false, 0);
            robot.ringShooter.launchRingAngularVelocity(10.35, false, 250);
        }
        robot.logAndPrint(this.getRuntime() - initialTime + " set powers zero");
        robot.ringShooter.setFlyWheelMotorPower(0);
        robot.ringShooter.setIntakeMotorPower(0);

    }

    public void logAndPrint(String text) {
        Robot.writeToMatchFile(text, false);
        addLine(text);
    }

    public void logAndPrint(String text, boolean includeTimeStamp) {
        Robot.writeToMatchFile(text, includeTimeStamp);
        addLine(text);
    }

    public void addLine(String telemetryText) {
        telemetry.addLine(telemetryText);
        telemetry.update();
    }

    private void driveToCenter(boolean goBack) {
        robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(0, 0)).build());

        if (goBack) {
            double delay = 2 * 1000;
            double waitUntil = getRuntime() + (double) (delay / 1000);
            while (getRuntime() < waitUntil) ;

            robot.drive(robot.trajectoryBuilder().lineToConstantHeading(new Vector2d(-61.125, -41.875)).build());
        }
    }


}

