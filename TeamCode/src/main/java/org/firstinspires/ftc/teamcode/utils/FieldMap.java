package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

import java.util.ArrayList;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

/**
 * Stores information about the field so that it can represent a digital "map" of the field
 */
public class FieldMap {

    private static final float mmTargetHeight   = (6) * (float)mmPerInch;
    private static final float halfField = 72 * (float)mmPerInch;
    private static final float quadField  = 36 * (float)mmPerInch;



    public FieldMap(){

    }

}
