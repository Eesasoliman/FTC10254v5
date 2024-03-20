package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.ColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;

@TeleOp
public class T extends DriveOpMode {
    @Override
    public void runOpMode() {
        try {
            telemetry.setMsTransmissionInterval(50);

            int i = 0;
            while (opModeInInit()) {
                telemetry.addData("id", i);
                telemetry.update();
                i++;
                sleep(10);
            }
        } catch (Exception e) {
            telemetry.addData("Error", e);
            telemetry.update();
        }
    }
}
