package org.firstinspires.ftc.teamcode.roadrunner;

import org.firstinspires.ftc.teamcode.resources.*;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
@Disabled
public class YesAuton extends DriveOpMode {
    @Override
    public void runOpMode() {
        telemetry.setMsTransmissionInterval(50);
        drive = initDriveOpMode();
        // Remove false when implementing with new robot
        boolean[] driveVariables = initWithController(false);

        int imageNum = 0;
        int multiplier = 1;
        if (driveVariables[0])
        {
            if (driveVariables[1])
            {
                drive.setPoseEstimate(new Pose2d(DriveOpMode.BACK_X, DriveOpMode.Y, Math.toRadians(90)));
            }
            else
            {
                drive.setPoseEstimate(new Pose2d(DriveOpMode.FRONT_X, DriveOpMode.Y, Math.toRadians(90)));
            }
            BlueColorPipeline pipeline = startBlueCamera();
            while (!isStopRequested() && opModeInInit())
            {
                sleep(1000);
                imageNum = pipeline.getImageNum();
                telemetry.addData("imageNum", imageNum);
                telemetry.update();
            }
        }
        else {
            multiplier *= -1;
            if (driveVariables[1])
            {
                drive.setPoseEstimate(new Pose2d(DriveOpMode.BACK_X, -DriveOpMode.Y, Math.toRadians(270)));
            }
            else
            {
                drive.setPoseEstimate(new Pose2d(DriveOpMode.FRONT_X, -DriveOpMode.Y, Math.toRadians(270)));
            }
            RedColorPipeline pipeline = startRedCamera();
            while (!isStopRequested() && opModeInInit()) {
                sleep(1000);
                imageNum = pipeline.getImageNum();
                telemetry.addData("imageNum", imageNum);
                telemetry.update();
            }
        }

        TrajectorySequence[] trajectories = getTrajectories(imageNum, multiplier);

        waitForStart();

        int liftHeight = 10;
        if (driveVariables[1])
        {
            if (imageNum == 1)
            {
                drive.followTrajectorySequence(trajectories[0]);
                intake();
                if (driveVariables[2])
                {
                    drive.followTrajectorySequence(trajectories[7]);
                    scorePixelsOnBackboard(liftHeight);
                    if (driveVariables[3])
                    {
                        for (int i = 0; i < 2; i++) {
                            drive.followTrajectorySequence(trajectories[9]);
                            intake();
                            drive.followTrajectorySequence(trajectories[8]);
                            scorePixelsOnBackboard(liftHeight);
                        }
                    }
                }
            }
            else if (imageNum == 2 || imageNum == 3 || imageNum == 4)
            {
                drive.followTrajectorySequence(trajectories[1]);
                intake();
                if (driveVariables[2])
                {
                    drive.followTrajectorySequence(trajectories[7]);
                    scorePixelsOnBackboard(liftHeight);
                    if (driveVariables[3])
                    {
                        for (int i = 0; i < 2; i++) {
                            drive.followTrajectorySequence(trajectories[9]);
                            intake();
                            drive.followTrajectorySequence(trajectories[8]);
                            scorePixelsOnBackboard(liftHeight);
                        }
                    }
                }
            }
            else if (imageNum == 5)
            {
                drive.followTrajectorySequence(trajectories[2]);
                intake();
                if (driveVariables[2])
                {
                    drive.followTrajectorySequence(trajectories[7]);
                    scorePixelsOnBackboard(liftHeight);
                    if (driveVariables[3])
                    {
                        for (int i = 0; i < 2; i++) {
                            drive.followTrajectorySequence(trajectories[9]);
                            intake();
                            drive.followTrajectorySequence(trajectories[8]);
                            scorePixelsOnBackboard(liftHeight);
                        }
                    }
                }
            }
        }
        else
        {
            if (imageNum == 1)
            {
                drive.followTrajectorySequence(trajectories[3]);
                intake();
                if (driveVariables[2])
                {
                    drive.followTrajectorySequence(trajectories[6]);
                    scorePixelsOnBackboard(liftHeight);
                    if (driveVariables[3])
                    {
                        for (int i = 0; i < 2; i++) {
                            drive.followTrajectorySequence(trajectories[9]);
                            intake();
                            drive.followTrajectorySequence(trajectories[8]);
                            scorePixelsOnBackboard(liftHeight);
                        }
                    }
                }
            }
            else if (imageNum == 2 || imageNum == 3 || imageNum == 4)
            {
                drive.followTrajectorySequence(trajectories[4]);
                intake();
                if (driveVariables[2])
                {
                    drive.followTrajectorySequence(trajectories[6]);
                    scorePixelsOnBackboard(liftHeight);
                    if (driveVariables[3])
                    {
                        for (int i = 0; i < 2; i++) {
                            drive.followTrajectorySequence(trajectories[9]);
                            intake();
                            drive.followTrajectorySequence(trajectories[8]);
                            scorePixelsOnBackboard(liftHeight);
                        }
                    }
                }
            }
            else if (imageNum == 5)
            {
                drive.followTrajectorySequence(trajectories[5]);
                intake();
                if (driveVariables[2])
                {
                    drive.followTrajectorySequence(trajectories[6]);
                    scorePixelsOnBackboard(liftHeight);
                    if (driveVariables[3])
                    {
                        for (int i = 0; i < 2; i++) {
                            drive.followTrajectorySequence(trajectories[9]);
                            intake();
                            drive.followTrajectorySequence(trajectories[8]);
                            scorePixelsOnBackboard(liftHeight);
                        }
                    }
                }
            }
        }
    }
}