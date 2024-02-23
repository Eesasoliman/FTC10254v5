package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.resources.RedColorPipeline;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class TestRedCameraPipeline extends DriveOpMode {
    int imageNum;

    @Override
    public void runOpMode()
    {
        robot.init(hardwareMap);
        RedColorPipeline pipeline = startRedCamera();
        long currentTimestamp = 0;
        boolean updated = false;
        int lh = 200;
        int ls = 70;
        int lv = 20;
        int uh = 240;
        int us = 100;
        int uv = 70;

        while (opModeInInit()) {
            if(gamepad1.x){
                lh+=5;
                updated = true;
                currentTimestamp = gamepad1.timestamp;
            }
            if (gamepad1.y){
                ls+=5;
                updated = true;
                currentTimestamp = gamepad1.timestamp;
            }
            if(gamepad1.b){
                lv+=5;
                updated = true;
                currentTimestamp = gamepad1.timestamp;
            }
            if(gamepad1.dpad_left){
                uh += 5;
                updated = true;
                currentTimestamp = gamepad1.timestamp;
            }
            if(gamepad1.dpad_up){
                us+=5;
                updated = true;
                currentTimestamp = gamepad1.timestamp;
            }
            if (gamepad1.dpad_right){
                uv+=5;
                updated = true;
                currentTimestamp = gamepad1.timestamp;
            }

            if (updated && gamepad1.timestamp > currentTimestamp + 300)
            {
                telemetry.addData("lowerBound", "H: " + lh + ", " + "S: " + ls + ", " + "V: " + lv);
                telemetry.addData("upperBound", "H: " + uh + ", " + "S: " + us + ", " + "V: " + uv);
                pipeline.setBounds(
                    new Scalar(179 * (lh/360d), 255 * (ls/100d), 255*(lv/100d)),
                    new Scalar(179 * (uh/360d), 255 * (us/100d), 255*(uv/100d))
                );
                updated = false;
            }

            sleep(5);
            imageNum = pipeline.getImageNum();
            telemetry.addData("imageNum", imageNum);
            telemetry.update();
        }
        waitForStart();

        telemetry.addLine("USE ONLY INIT FOR TESTING");
        telemetry.update();
    }
}
