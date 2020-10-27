package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drives.MecanumDrive;

public class RobotWood extends Robot {

    public RobotWood(HardwareMap hw){
        super(hw);
        drive = new MecanumDrive(hw);
    }
}
