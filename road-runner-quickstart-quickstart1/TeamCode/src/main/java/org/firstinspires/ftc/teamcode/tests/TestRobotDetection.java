package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.resources.DriveOpMode;

@Autonomous
public class TestRobotDetection extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        boolean isBlueSide = false;

        telemetry.addLine("DO NOT CLICK START.");
        telemetry.addLine("Waiting for camera processor to start streaming...");
        telemetry.update();
        waitForAprilTagCamera();

        telemetry.addLine("DO NOT CLICK START.");
        telemetry.addLine("Starting detection (5 seconds max).");
        telemetry.addData("isBlueSide", isBlueSide);
        telemetry.update();

        if (detectRobot(isBlueSide)) {
            telemetry.addLine("Detected robot or did not detect one tag.");
        } else
        {
            telemetry.addLine("No robot detected. All april tags detected.");
        }
        telemetry.addLine("Press start to clear telemetry and end program.");
        telemetry.update();
        waitForStart();
    }
}