package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.mechanisms.GoalLift;
import org.firstinspires.ftc.teamcode.robots.Robot;
import org.firstinspires.ftc.teamcode.robots.RobotTechnicolorRR;
import org.firstinspires.ftc.teamcode.utils.FieldMap;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;
import org.firstinspires.ftc.teamcode.utils.SoundLibrary;
import org.firstinspires.ftc.teamcode.utils.Vuforia;
import org.firstinspires.ftc.teamcode.utils.VuforiaLocalization;

import java.util.List;

@TeleOp (name = "Technicolor", group = "Competition")
public class TeleOpTechnicolor extends LinearOpMode {

    RobotTechnicolorRR robot;

    final double LIFT_POWER = 0.7;
    final double SHOOTER_POWER = 0.85;
    final double INTAKE_POWER = 1.0;

    final double MAX_DRIVE_SPEED = 0.65;
    final double MIN_DRIVE_SPEED = 0.35;

    final double MAX_TURN_SPEED = 0.5;
    final double MIN_TURN_SPEED = 0.2;

    final long LIFT_TIME_LIMIT = 500;
    final long LOWER_TIME_LIMIT = 500;

    private double driveMult = MIN_DRIVE_SPEED;
    private double turnMult = MIN_TURN_SPEED;

    double velocity = 10;
    double velocityChange = 0.25;
    double velocitySmallChange = 0.1;

    double maxVelocity = 9.9;
    double minVelocity = 9.5;

    boolean isPickingUpRing = false;
    boolean isRingStuck = false;

    int ringCounter = 0;

    private GamepadEvents gamepad1;
    private GamepadEvents gamepad2;
    private Vuforia vuforia = Vuforia.getInstance();
    private VuforiaLocalization vuforiaLocalizer;

    private final String VUFORIA_TRACKABLES_ASSET_NAME = "UltimateGoal";

    private Thread shootPowershotThread;
    private Thread shootInGoal;

    @Override
    public void runOpMode() throws InterruptedException {

        Robot.createMatchLogFile( this.getClass().getSimpleName() );

        robot = new RobotTechnicolorRR(hardwareMap, this);

        SoundLibrary.playRandomStartup();

        gamepad1 = new GamepadEvents(super.gamepad1);
        gamepad2 = new GamepadEvents(super.gamepad2);
        /*if(!Vuforia.getInstance().isRunning()) {
            Vuforia.getInstance().start();
        }*/
        /*vuforiaLocalizer = new VuforiaLocalization(VUFORIA_TRACKABLES_ASSET_NAME);
        vuforiaLocalizer.activateTracking();*/

        robot.ringShooter.setPusherPosition(Robot.PUSHER_RETRACTED);
        robot.goalLift.setClawPosition(Robot.CLAW_OPEN);

        shootPowershotThread = new Thread(() -> {
            /*robot.setPosition(new Pose2d(0, 18, 0)); // 61, 14 if the robot is 18" wide: 70.125-robotwidth/2, 23.125-robotwidth/2
            robot.drive(robot.trajectoryBuilder().addDisplacementMarker(() -> robot.ringShooter.setFlyWheelMotorVelocity(9.4, AngleUnit.RADIANS)).splineToConstantHeading(new Vector2d(-4, -20.5), 0).build());
            robot.drive.turn(Math.toRadians(0));
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, false);
            robot.drive.turn(Math.toRadians(6));
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
            robot.drive.turn(Math.toRadians(6));
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);*/
            robot.drive.turn(Math.toRadians(0));
            telemetry.addData("Angle", robot.drive.getExternalHeading());
            telemetry.addData("Flywheel Velocity", robot.ringShooter.getFlyWheelVelocity(false));
            telemetry.update();
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_RIGHT_POWERSHOT, false, false);
            robot.drive.turn(Math.toRadians(6));
            telemetry.addData("Angle", robot.drive.getExternalHeading());
            telemetry.addData("Flywheel Velocity", robot.ringShooter.getFlyWheelVelocity(false));
            telemetry.update();
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_MIDDLE_POWERSHOT, false, false);
            robot.drive.turn(Math.toRadians(6));
            telemetry.addData("Angle", robot.drive.getExternalHeading());
            telemetry.addData("Flywheel Velocity", robot.ringShooter.getFlyWheelVelocity(false));
            telemetry.update();
            robot.shootAtTarget(FieldMap.ScoringGoals.RED_LEFT_POWERSHOT, true, false);
        });

        shootInGoal = new Thread(() -> {
            robot.ringShooter.setFlyWheelPID(new PIDFCoefficients(5, 0, 2, 12.5 * 12 / robot.batteryVoltageSensor.getVoltage()));
            robot.ringShooter.setFlyWheelMotorVelocity(9.9, AngleUnit.RADIANS);
            //robot.drive.turnTo(0);
            for(int i = 0; i < 3; i++) {
                robot.ringShooter.pushRingTime();
                sleep(300);
            }
            ringCounter = 0;
        });

        Robot.writeToMatchFile( "Initialization Complete", true );

        telemetry.addLine("Initialization Complete");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            //Sprint control
            driveMult = gamepad1.left_stick_button.getValue() ? MAX_DRIVE_SPEED : MIN_DRIVE_SPEED;
            turnMult = gamepad1.right_stick_button.getValue() ? MAX_TURN_SPEED : MIN_TURN_SPEED;

            if(!shootPowershotThread.isAlive())
                robot.teleOpDrive(-gamepad1.left_stick_y*driveMult, gamepad1.left_stick_x*driveMult, -gamepad1.right_stick_x*turnMult);

            /*if( gamepad2.y.onPress() )
                velocity = maxVelocity;
            else if( gamepad2.a.onPress() )
                velocity = minVelocity;*/

            /*
            if( gamepad2.x.onPress() )
                robot.ringBlocker.setBlockerPositionAsync( Robot.BLOCKER_BLOCKED );
            else if( gamepad2.b.onPress() )
                robot.ringBlocker.setBlockerPositionAsync( Robot.BLOCKER_RETRACTED );
             */

            // increases and decreases the velocity of the flyWheels
            if( gamepad1.dpad_up.onPress() || gamepad2.dpad_up.onPress() )
                velocity += velocityChange;
            else if( gamepad1.dpad_down.onPress() || gamepad2.dpad_down.onPress() )
                velocity -= velocityChange;
            else if( gamepad1.dpad_right.onPress() || gamepad2.dpad_right.onPress() )
                velocity += velocitySmallChange;
            else if( gamepad1.dpad_left.onPress() || gamepad2.dpad_left.onPress() )
                velocity -= velocitySmallChange;

            if(gamepad2.dpad_up.onHeldFor(500))
                velocity = maxVelocity;
            if(gamepad2.dpad_down.onHeldFor(500))
                velocity = minVelocity;

            telemetry.addLine( "velocity: " + velocity );

            // claw position
            if(gamepad1.x.onPress() || gamepad2.x.onPress())
                robot.goalLift.setClawPosition( GoalLift.ClawPosition.CLOSED );
            if(gamepad1.b.onPress() || gamepad2.b.onPress())
                robot.goalLift.setClawPosition( GoalLift.ClawPosition.OPEN );

            // goal lift
            if( gamepad1.y.onPress() || gamepad2.y.onPress() ) {
                //if the goal lift is running but hasn't been lowered all the way yet
                if(robot.goalLift.goalLiftIsRunning() && robot.goalLift.getCurrentLiftPosition() == GoalLift.LiftPosition.LIFTED) {
                    robot.goalLift.stopGoalLift();
                    robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LIFTED, LIFT_POWER + 0.3, LIFT_TIME_LIMIT );
                }
                else if(robot.goalLift.goalLiftIsRunning()) {
                    robot.goalLift.stopGoalLift();
                }
                else {
                    robot.goalLift.setGoalLiftPositionAsync( GoalLift.LiftPosition.LIFTED, LIFT_POWER + 0.3, LIFT_TIME_LIMIT );
                }
            }
            if( gamepad1.a.onPress() || gamepad2.a.onPress() ) {
                //if the goal lift is running but hasn't been raised all the way yet
                if(robot.goalLift.goalLiftIsRunning() && robot.goalLift.getCurrentLiftPosition() == GoalLift.LiftPosition.LOWERED) {
                    robot.goalLift.stopGoalLift();
                    robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LOWERED, LIFT_POWER, LOWER_TIME_LIMIT);
                }
                else if(robot.goalLift.goalLiftIsRunning()) {
                    robot.goalLift.stopGoalLift();
                }
                else {
                    robot.goalLift.setGoalLiftPositionAsync(GoalLift.LiftPosition.LOWERED, LIFT_POWER, LOWER_TIME_LIMIT);
                }
            }

            // ring shooter = gamepad1.right_trigger
            if(!shootPowershotThread.isAlive() && !shootInGoal.isAlive() && (gamepad1.left_trigger.onPress() || gamepad2.left_trigger.onPress()))
                shootPowershotThread.start();
                //shoot powershots
                //robot.ringShooter.setFlyWheelMotorVelocity((gamepad1.left_trigger.getTriggerValue() + gamepad2.left_trigger.getTriggerValue()) * velocity, AngleUnit.RADIANS);

            // ring pusher (servo) = gamepad1.left_bumper
            if( (gamepad1.right_trigger.onPress() || gamepad2.right_trigger.onPress()) && !shootPowershotThread.isAlive() && !shootInGoal.isAlive())
                shootInGoal.start();
                //robot.ringShooter.pushRingTimeAsync();

            // intake = gamepad1.left_trigger
            if(gamepad1.right_bumper.onPress() || gamepad2.right_bumper.onPress())
                robot.ringShooter.setIntakeMotorPower( robot.ringShooter.getIntakePower() > 0 ? 0 : INTAKE_POWER);
            else if(gamepad1.left_bumper.onPress() || gamepad2.left_bumper.onPress())
                robot.ringShooter.setIntakeMotorPower(robot.ringShooter.getIntakePower() < 0 ? 0 : -INTAKE_POWER);
            // robot.ringShooter.setIntakeMotorPower( gamepad1.left_trigger*INTAKE_POWER );

            // addMotorInfoTelemetry();

            // addControlTelemetry();

            // addDriveInfoTelemetry();

            if(gamepad1.back.onPress()) {
                if(shootPowershotThread.isAlive()) {
                    shootPowershotThread.interrupt();
                } else {
                    shootPowershotThread.start();
                }
            }

            //if the robot is picking up a ring
            if(robot.intakeSensor.isPickingUpRing() && !isPickingUpRing) {
                isPickingUpRing = true;
            }
            //else if the robot was picking up a ring, the ring should have made it to the top
            else if(!robot.intakeSensor.isPickingUpRing() && isPickingUpRing) {
                ringCounter += 1;
                isPickingUpRing = false;
            }

            if(ringCounter >= 2) {
                robot.ringShooter.setFlyWheelMotorVelocity(9.9, AngleUnit.RADIANS);
                if(ringCounter == 3) {
                    robot.setIntakePower(0);
                }
            }
            else {
                robot.ringShooter.setFlyWheelMotorVelocity(0, AngleUnit.RADIANS);
                robot.setIntakePower(1);
            }

            if(robot.intakeSensor.isRingStuck() && !isRingStuck) {
                robot.ringShooter.setIntakeMotorPower(-1);
                isRingStuck = true;
            }
            else if(!robot.intakeSensor.isRingStuck() && isRingStuck) {
                isRingStuck = false;
            }

            telemetry.addLine( "Left Fly Wheel Velocity  = " + robot.ringShooter.getFlyWheelVelocity( true ) );
            telemetry.addLine( "Right Fly Wheel Velocity = " + robot.ringShooter.getFlyWheelVelocity( false ) );

            telemetry.addData("Number of Rings", ringCounter);

            telemetry.addData("Intake Current Draw", robot.intakeSensor.getCurrent(CurrentUnit.AMPS));

            //telemetry.addData("Vuforia Position", FieldMap.toPose2d(vuforiaLocalizer.getRobotPosition(), vuforiaLocalizer.getRobotRotation()));

            //vuforiaLocalizer.updateRobotLocation();
            telemetry.update();
            gamepad1.update();
            gamepad2.update();
            robot.drive.update();
            //vuforiaLocalizer.updateRobotLocation();

            if(isStopRequested())
                if(Vuforia.getInstance().isRunning())
                    Vuforia.getInstance().close();
        }
    }


    public void addControlTelemetry() {

        telemetry.addLine("            Controls:");
        telemetry.addData("Drive ", "Gp1: left stick y (axis)")
                .addData("Strafe", "Gp1: left stick x (axis)")
                .addData("Rotate", "Gp1: right stick x (axis)")
                .addData("Open Claw ", "Gp1: b")
                .addData("Close Claw", "Gp1: x")
                .addData("Lift Goal Lift ", "Gp1: y")
                .addData("Lower Goal Lift", "Gp1: a")
                .addData("Flywheels", "Gp1: right trigger")
                .addData("Ring Pusher", "Gp1/Gp2: left bumper")
                .addData("Intake Toggle", "Gp1/Gp2: right bumper")
                .addData("Negate Intake", "Gp1/Gp2: left trigger")
                .addData("+/- flywheel velocity by " + velocityChange, "Gp1: dpad up/down")
                .addData("+/- flywheel velocity by " + velocitySmallChange, "Gp1: dpad right/left")
                .addData(" ", " ");
        addLine();
    }

    public void addInfoTelemetry() {

        telemetry.addLine("left_stick_y  = " + gamepad1.left_stick_y );
        telemetry.addLine("left_stick_x  = " + gamepad1.left_stick_x );
        telemetry.addLine("right_stick_x = " + gamepad1.right_stick_x );
        addLine();

        //telemetry.addLine("longitudinal position = " + robot.tracker.getLongitudinalPosition() + " (ticks), " + robot.mecanumDrive.convertTicksDist( robot.tracker.getLongitudinalPosition()) + " (in)" );
        //telemetry.addLine("lateral position = " + robot.tracker.getLateralPosition() + " (ticks), " + robot.mecanumDrive.convertTicksDist( robot.tracker.getLateralPosition()) + " (in)" );
        addLine();

        telemetry.addLine( "Claw Position = " + robot.goalLift.getClawLocation() + " :: "  + robot.goalLift.getClawPosition() );
        telemetry.addLine( "Lift Position = " + robot.goalLift.getCurrentLiftPosition() + " :: " + robot.goalLift.getLiftPower() );
        addLine();

        telemetry.addLine( "Average Shooter Power = " + (robot.ringShooter.getFlyWheelPower(true)+robot.ringShooter.getFlyWheelPower(false))/2 );
        addLine();

        telemetry.addLine( "Pusher Position = " + robot.ringShooter.getPusherLocation() + " :: " + robot.ringShooter.getPusherPosition() );
        addLine();

        telemetry.addLine( "Intake Power = " + robot.ringShooter.getIntakePower() );
        addLine();

        //telemetry.addLine("getGyroHeading    = " + robot.tracker.getGyroHeading() );
        //telemetry.addLine("get360GyroHeading = " + robot.tracker.get360GyroHeading() );

    }

    public void addLine() {
        telemetry.addLine();
    }

    public void addMotorInfoTelemetry() {

        //(leftFront, leftRear, rightRear, rightFront)

        List<Double> positions = robot.drive.getWheelPositions();

        telemetry.addData( "leftFront", "Pos 0: " + positions.get( 0 ) );
        telemetry.addData( "leftRear", "Pos 1: " + positions.get( 1 ) );
        telemetry.addData( "rightRear", "Pos 2: " + positions.get( 2 ) );
        telemetry.addData( "rightFront", "Pos 3: " + positions.get( 3 ) );

        List<Double> velocities = robot.drive.getWheelVelocities();

        addLine();

        telemetry.addData( "leftFront", "Vel 0: " + velocities.get( 0 ) );
        telemetry.addData( "leftRear", "Vel 1: " + velocities.get( 1 ) );
        telemetry.addData( "rightRear", "Vel 2: " + velocities.get( 2 ) );
        telemetry.addData( "rightFront", "Vel 3: " + velocities.get( 3 ) );

        addLine();

    }

    public void addDriveInfoTelemetry() {

        telemetry.addLine( "Gp1: left_stick_x :: " + gamepad1.left_stick_x );
        telemetry.addLine( "Gp1: left_stick_y :: " + gamepad1.left_stick_y );
        telemetry.addLine( "Gp1: right_stick_y :: " + gamepad1.right_stick_y );

        addLine();

        telemetry.addLine( "drive multiplier " + driveMult );
        telemetry.addLine( "turn multiplier " + turnMult );

        addLine();

        telemetry.addLine( "drive input :: " + -gamepad1.left_stick_y*driveMult );
        telemetry.addLine( "strafe input :: " + gamepad1.left_stick_x*driveMult );
        telemetry.addLine( "rotate input :: " + -gamepad1.right_stick_x*turnMult );


    }

}
