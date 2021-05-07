package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSensor {

    private final double PICKUP_RING_CURRENT_DRAW = 1.8;
    private final double STUCK_RING_CURRENT_DRAW = 3;
    private final double CURRENT_CHECK_TIME = 700;
    private final double DOUBLE_RING_PICKUP_TIME = 800;

    private final double RING_TRAVEL_TIME = 1000;

    private boolean isPickingUp = false;

    private double ringPickupCurrentElapsedTime;
    private double ringStuckCurrentElapsedTime;

    private Thread intakeThread;

    private LinearOpMode opMode;

    private DcMotorEx intakeMotor;

    public IntakeSensor(DcMotorEx intakeMotor, LinearOpMode opMode) {
        this.intakeMotor = intakeMotor;
        this.opMode = opMode;
        intakeThread = new Thread(() -> {
            while (!this.opMode.isStarted());
            while (this.opMode.opModeIsActive()) {
                if(isStuckRingCurrentDraw()) {
                    ringStuckCurrentElapsedTime += ringStuckCurrentElapsedTime > 0 ? System.currentTimeMillis() - ringStuckCurrentElapsedTime : System.currentTimeMillis();
                } else if(isPickupRingCurrentDraw()) {
                    ringPickupCurrentElapsedTime += ringPickupCurrentElapsedTime > 0 ? System.currentTimeMillis() - ringPickupCurrentElapsedTime : System.currentTimeMillis();
                } else {
                    ringStuckCurrentElapsedTime = 0;
                    ringPickupCurrentElapsedTime = 0;
                }
            }
        });
        intakeThread.start();
    }

    public double getCurrent(CurrentUnit unit) {
        return intakeMotor.getCurrent(unit);
    }

    public boolean isPickingUpRing() {
        return ringPickupCurrentElapsedTime > CURRENT_CHECK_TIME;
    }

    public boolean isRingStuck() {
        return ringStuckCurrentElapsedTime > CURRENT_CHECK_TIME;
    }

    private boolean isPickupRingCurrentDraw() {
        return getCurrent(CurrentUnit.AMPS) > PICKUP_RING_CURRENT_DRAW;
    }

    private boolean isStuckRingCurrentDraw() {
        return getCurrent(CurrentUnit.AMPS) > STUCK_RING_CURRENT_DRAW;
    }
}
