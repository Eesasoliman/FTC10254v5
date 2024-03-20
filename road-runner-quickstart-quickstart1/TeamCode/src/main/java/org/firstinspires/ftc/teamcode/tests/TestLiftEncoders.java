package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.resources.DriveOpMode;

@TeleOp
public class TestLiftEncoders extends DriveOpMode {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.LL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.RL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("LL", robot.LL.getCurrentPosition());
            telemetry.addData("RL", robot.RL.getCurrentPosition());
//            lift(3);
            telemetry.update();
        }
    }
}
