package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestDriveToBackboard extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();
        startCameras(false);
        closeColorPipelineCamera();

        sleep(1000);
        Pose2d back = new Pose2d(50, -36, Math.toRadians(0));
        Pose2d currentPose = relocalize(false);
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