package org.firstinspires.ftc.teamcode.resources;

import org.opencv.core.Scalar;

public class RedColorPipeline extends ColorPipeline {
    public RedColorPipeline() {
        super(
                new Scalar(179 * (0/360d), 255 * (85/100d), 255*(30/100d)),
                new Scalar(179 * (355/360d), 255 * (95/100d), 255*(95/100d))
        );
    }
}
