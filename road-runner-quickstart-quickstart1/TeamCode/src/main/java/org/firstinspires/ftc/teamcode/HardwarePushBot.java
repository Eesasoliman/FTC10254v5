package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

public class HardwarePushBot {
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotorEx FLD   = null;
    public DcMotorEx FRD  = null;
    public DcMotorEx BLD   = null;
    public DcMotorEx BRD  = null;

    public DcMotorEx LL  = null;
    public DcMotorEx RL  = null;
    public DcMotorEx IN  = null;

    public Servo   RFS = null;
    public Servo   LFS = null;

    public Servo   FPS = null;
    public Servo   BPS = null;

    public Servo AP = null;

//    public ColorSensor CS = null;
//    public ColorSensor CSL = null;
//
//    public DistanceSensor DSRF = null;
//    public DistanceSensor DSRB = null;
//    public DistanceSensor DSLF = null;
//    public DistanceSensor DSLB = null;
//    public DistanceSensor DSCL = null;
//    public DistanceSensor DSCR = null;

    // Deadwheels (Odometry Wheels)
    public DcMotorEx DWR = null;
    public DcMotorEx DWL = null;
    public DcMotorEx DWC = null;

    HardwareMap hardwareMap = null;
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public HardwarePushBot() {}

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap cMap)    {
        hardwareMap = cMap;

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        FLD  = hardwareMap.get(DcMotorEx.class, "FLD");
        FRD = hardwareMap.get(DcMotorEx.class, "FRD");
        BLD = hardwareMap.get(DcMotorEx.class, "BLD");
        BRD = hardwareMap.get(DcMotorEx.class, "BRD");

        LL = hardwareMap.get(DcMotorEx.class, "LL");
        RL = hardwareMap.get(DcMotorEx.class, "RL");
        IN = hardwareMap.get(DcMotorEx.class, "IN");

        // Dead Wheels (Odometry Wheels/Encoders)
        DWR = hardwareMap.get(DcMotorEx.class, "FRD");
        DWL = hardwareMap.get(DcMotorEx.class, "BLD");
        DWC = hardwareMap.get(DcMotorEx.class, "FLD");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLD.setDirection(DcMotorEx.Direction.REVERSE);
        FRD.setDirection(DcMotorEx.Direction.FORWARD);
        BLD.setDirection(DcMotorEx.Direction.REVERSE);
        BRD.setDirection(DcMotorEx.Direction.FORWARD);

        LL.setDirection(DcMotorEx.Direction.FORWARD);
        RL.setDirection(DcMotorEx.Direction.REVERSE);
        IN.setDirection(DcMotorEx.Direction.REVERSE);

        FLD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FRD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BLD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BRD.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LFS = hardwareMap.get(Servo.class, "LFS");
        RFS = hardwareMap.get(Servo.class, "RFS");
        FPS = hardwareMap.get(Servo.class, "FPS");
        BPS = hardwareMap.get(Servo.class, "BPS");
        AP = hardwareMap.get(Servo.class, "AP");

//        DSRF = hardwareMap.get(DistanceSensor.class, "DSRF");
//        CS = hardwareMap.get(ColorSensor.class, "CS");


//        RL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        LL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}