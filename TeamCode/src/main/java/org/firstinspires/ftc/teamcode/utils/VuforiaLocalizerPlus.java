package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class VuforiaLocalizerPlus extends VuforiaLocalizerImpl {
    public VuforiaLocalizerPlus(Parameters parameters) {
        super(parameters);
    }

    public void close() {
        super.close();
    }
}
