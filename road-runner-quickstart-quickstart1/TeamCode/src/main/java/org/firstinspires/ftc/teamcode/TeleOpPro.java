package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpPro", group = "Drive")
public class TeleOpPro extends LinearOpMode {

    // Class Gamepad Documentation:
    // https://ftctechnh.github.io/ftc_app/doc/javadoc/com/qualcomm/robotcore/hardware/Gamepad.html

    HardwarePushBot robot = new HardwarePushBot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        robot.FLD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.FRD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.BLD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.BRD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        robot.LL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        robot.RL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.setMsTransmissionInterval(50);

        waitForStart();
        // Initialize Servo Positions
        robot.LFS.setPosition(0.95);
        robot.RFS.setPosition(0.05);

        robot.CLAW.setPosition(0);
//        robot.FPS.setPosition(0);
//        robot.BPS.setPosition(0);

        double speed = 1.0;

        double lefty;
        double righty;
        double leftx;
        double rightx;

        boolean buttonL;
        boolean buttonR;

        double triggerR;
        double triggerL;

        double intakeSpeed;
        double intakeSpeedMultiplier = 1;

        double armPower;
        double armSpeed = 1.0;
        double dropdownPos = 0;
        double prevDropdownPos = 0;
//        double liftPosition = 0.0;

        boolean dpadUp;
        boolean dpadDown;
        boolean dpadRight;
        boolean dpadLeft;

        boolean a;
        boolean b;
        boolean x;
        boolean y;

        long nextWristTimestamp = 0;
        long nextClawTimestamp = 0;
        boolean isClawOpen = true;
        boolean isWristVertical = true;

        while (opModeIsActive()) {
            // Gamepad 1
            lefty = -gamepad1.left_stick_y;
            righty = -gamepad1.right_stick_y;
            leftx = gamepad1.left_stick_x;
            rightx = gamepad1.right_stick_x;

            // Gamepad 2
            armPower = gamepad2.left_stick_y;
            intakeSpeed = gamepad2.right_stick_y;

            buttonL = gamepad2.left_bumper;
            buttonR = gamepad2.right_bumper;

            triggerL = gamepad2.left_trigger;
            triggerR = gamepad2.right_trigger;

            dpadUp = gamepad2.dpad_up;
            dpadLeft = gamepad2.dpad_left;
            dpadRight = gamepad2.dpad_right;
            dpadDown = gamepad2.dpad_down;

            y = gamepad2.y;
            x = gamepad2.x;
            a = gamepad2.a;
            b = gamepad2.b;

            // Gamepad 1 Code
            if (gamepad1.a) {
                speed = 1.0;
            } else if (gamepad1.b) {
                speed = 0.75;
            } else if (gamepad1.x) {
                speed = 0.50;
            } else if (gamepad1.y) {
                speed = 0.30;
            }

            // Gamepad 2 Code
            if (buttonR && gamepad2.timestamp > nextClawTimestamp) {
                // Control CLAW
                nextClawTimestamp = gamepad2.timestamp + 300;

                if (isClawOpen) {
                    // Set CLAW to close position
                    robot.CLAW.setPosition(0);
                } else {
                    // Set CLAW to open position
                    robot.CLAW.setPosition(.4);
                }
                isClawOpen = !isClawOpen;
            }

            if (buttonL && gamepad2.timestamp > nextWristTimestamp) {
                // Rotate WRIST
                nextWristTimestamp = gamepad2.timestamp + 300;

                if (isWristVertical) {
                    // Set WRIST to horizontal position
                    robot.WRIST.setPosition(0);
                } else {
                    // Set WRIST to vertical position
                    robot.WRIST.setPosition(.4);
                }
                isWristVertical = !isWristVertical;
            }

            if (triggerL > 0) {
                // Swivel in
                robot.LFS.setPosition(0.95); // To swivel in more, increase this
                robot.RFS.setPosition(0.05);// To swivel in more, decrease this
            }

            if (triggerR > 0) {
                // Swivel out
                robot.LFS.setPosition(0.50); // To swivel out more, decrease this
                robot.RFS.setPosition(0.50); // To swivel out more, increase this
            }

            if (dpadUp && dropdownPos < 4) {
                dropdownPos++;
            } else if (dpadLeft) {
                intakeSpeedMultiplier = 0.5;
            } else if (dpadRight) {
                intakeSpeedMultiplier = 0.8;
            } else if (dpadDown && dropdownPos > 0) {
                dropdownPos--;
            }

            if (a) {
                robot.AP.setDirection(Servo.Direction.REVERSE);
                robot.AP.setPosition(0);
            }

            if (b) {
                robot.AP.setDirection(Servo.Direction.REVERSE);
                robot.AP.setPosition(1);
            }

            if (x) {
                // Set CLAW to open position
                robot.CLAW.setPosition(.4);
                // Set WRIST to vertical position
                robot.WRIST.setPosition(.4);
                // Swivel in
                robot.LFS.setPosition(0.95); // To swivel in more, increase this
                robot.RFS.setPosition(0.05);// To swivel in more, decrease this
            }

            // Gamepad 1
            double FLDp = lefty + leftx;
            double FRDp = righty - rightx;
            double BLDp = lefty - leftx;
            double BRDp = righty + rightx;

            robot.FLD.setPower(speed * FLDp);
            robot.FRD.setPower(speed * FRDp);
            robot.BLD.setPower(speed * BLDp);
            robot.BRD.setPower(speed * BRDp);

            // Gamepad 2
            if (dropdownPos != prevDropdownPos) {
                if (dropdownPos == 0) {
                    robot.DROPDOWN.setPosition(0.00);
                } else if (dropdownPos == 1) {
                    robot.DROPDOWN.setPosition(0.25);
                } else if (dropdownPos == 2) {
                    robot.DROPDOWN.setPosition(0.50);
                } else if (dropdownPos == 3) {
                    robot.DROPDOWN.setPosition(0.75);
                } else if (dropdownPos == 4) {
                    robot.DROPDOWN.setPosition(1.00);
                }
                prevDropdownPos = dropdownPos;
            }
            robot.IN.setPower(intakeSpeed * intakeSpeedMultiplier);
            robot.LL.setPower(armSpeed * armPower);
            robot.RL.setPower(armSpeed * armPower);

            telemetry.update();
        }
    }
}