package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TestTrajectorySequence extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        SampleMecanumDrive drive = initDriveOpMode();
        initAprilTagProcessor();

        sleep(2000);
        Pose2d startPose = new Pose2d(24, 36, Math.toRadians(0));
        Pose2d back = new Pose2d(50, 36, Math.toRadians(0));
        Pose2d correctPose = relocalize(startPose, 0);
        TrajectorySequence Test = drive.trajectorySequenceBuilder(correctPose)
                .lineToLinearHeading(back)
                .build();

        waitForStart();
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(Test);
    }
}