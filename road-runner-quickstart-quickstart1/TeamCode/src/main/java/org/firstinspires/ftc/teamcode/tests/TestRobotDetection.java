package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestRobotDetection extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();

        telemetry.addLine("DO NOT CLICK START.");
        telemetry.addLine("Building camera processor.");
        telemetry.update();
        initAprilTagProcessor();
        sleep(1000);

        telemetry.addLine("DO NOT CLICK START.");
        telemetry.addLine("Starting detection (5 seconds max).");
        telemetry.update();

        if (detectRobot(false)) {
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