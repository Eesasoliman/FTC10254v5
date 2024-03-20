package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
//@Disabled
public class TestDriveToBackboard extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();
        telemetry.addLine("Cameras...");
        telemetry.update();
        startCameras(false);
        closeColorPipelineCamera();

        telemetry.addLine("Starting...");
        telemetry.update();
        sleep(1000);
        telemetry.addLine("Localizing...");
        telemetry.update();
        Pose2d back = new Pose2d(50, -36, Math.toRadians(0));
        Pose2d currentPose = relocalize(false);
        telemetry.addLine("Position Returned...");
        telemetry.update();
        telemetry.addData("pose",currentPose);
        TrajectorySequence Test = drive.trajectorySequenceBuilder(currentPose)
                .lineToLinearHeading(back)
                .build();

        telemetry.update();

        waitForStart();

        drive.setPoseEstimate(currentPose);
        drive.followTrajectorySequence(Test);
    }
}
