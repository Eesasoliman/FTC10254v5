package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.resources.HardwarePushBot;
import org.firstinspires.ftc.teamcode.resources.DriveOpMode;

@TeleOp
@Disabled
public class TestDeadWheels extends DriveOpMode {
    HardwarePushBot robot = new HardwarePushBot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.DWR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.DWL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.DWC.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        robot.DWR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.DWL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        robot.DWC.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            final double countPerRevolution = 8192;

            telemetry.addLine("The following info is reported in the order DWR, DWL, and finally DWC");
            for (DcMotorEx deadWheel : new DcMotorEx[]{robot.DWR, robot.DWL, robot.DWC})
            {
                int pos = deadWheel.getCurrentPosition();
                double revs = pos/countPerRevolution;
                double angle = (revs * 360) % 360;

                telemetry.addData("Position", pos);
                telemetry.addData("Revolutions", revs);
                telemetry.addData("Normalized Angle", angle);
                telemetry.addLine("");
            }
            telemetry.update();
        }
    }
}
