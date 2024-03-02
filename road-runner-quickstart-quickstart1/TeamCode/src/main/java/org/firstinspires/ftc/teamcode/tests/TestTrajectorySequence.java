package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
@Disabled
public class TestTrajectorySequence extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);

        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        TrajectorySequence Test = drive.trajectorySequenceBuilder(startPose)
                .UNSTABLE_addTemporalMarkerOffset(0,() -> {
                    intake();
                    scorePixelsOnBackboard(12);
                })
                .forward(12)
                .back(12)
                .build();

        waitForStart();
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequence(Test);
    }
}