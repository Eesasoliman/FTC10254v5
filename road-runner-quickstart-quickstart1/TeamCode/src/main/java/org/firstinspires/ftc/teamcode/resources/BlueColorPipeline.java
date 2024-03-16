package org.firstinspires.ftc.teamcode.resources;

import org.opencv.core.Scalar;

public class BlueColorPipeline extends ColorPipeline {
    public BlueColorPipeline() {
        super(
                new Scalar(179 * (200/360d), 255 * (70/100d), 255*(20/100d)),
                new Scalar(179 * (240/360d), 255 * (100/100d), 255*(70/100d))
        );
    }
}
