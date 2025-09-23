package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.utils.PIDController;

@Config
public class DriveSubsystem {
    private static IMU imu;
    private static DcMotorEx lf, rf, lb, rb;
    private static PIDController headingPID;
    private static double targetHeading = 0.0; // radians
    // dashboard-tunable variables
    public static Boolean reverseDirections = true;
    public static Boolean FieldOriented = true;
    public static double SpeedModifier = 1.0;
    public static double Deadband = 0.05;
    static boolean wasTwisting = false;
    // PID coefficients (for heading hold)
    public static double kP = 0.5;
    public static double kI = 0.0;
    public static double kD = 0.05;

    public static void initialize(HardwareMap hwMap, Telemetry telemetry) {
        lf = hwMap.get(DcMotorEx.class, "left_front_motor");
        rf = hwMap.get(DcMotorEx.class, "right_front_motor");
        lb = hwMap.get(DcMotorEx.class, "left_back_motor");
        rb = hwMap.get(DcMotorEx.class, "right_back_motor");

        imu = hwMap.get(IMU.class, "gyro");

        // Initialize the heading PID controller
        headingPID = new PIDController(kP, kI, kD, true);
        headingPID.setOutputLimits(-1, 1);
        targetHeading = 0.0;

        // IMU parameters
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );

        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Disable built-in velocity control
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse motor directions depending on flag
        if (reverseDirections) {
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            lb.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            rf.setDirection(DcMotorEx.Direction.REVERSE);
            rb.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }

    public void handleControllerInput(Gamepad gamepad) {
        // Update PID gains from dashboard live
        headingPID.setGains(kP, kI, kD);

        double rawDrive  = -gamepad.left_stick_y;   // forward/backward
        double rawStrafe =  gamepad.left_stick_x;   // left/right
        double rawTwist  = -gamepad.right_stick_x;  // rotation

        double drive = Utils.applyDeadband(rawDrive, 0.05);
        double strafe = Utils.applyDeadband(rawStrafe, 0.05);
        double twist  = Utils.applyDeadband(rawTwist, 0.05);

        if (Math.abs(twist) > 0.05) {
            if (!wasTwisting) {
                targetHeading = getHeading();
                wasTwisting = true;
            }
        } else {
            twist = headingPID.output(targetHeading, getHeading());
            wasTwisting = false;
        }

        if (!FieldOriented) {
            // Robot-oriented drive
            double leftFrontInput  = drive + strafe + twist;
            double rightFrontInput = drive - strafe - twist;
            double leftBackInput   = drive - strafe + twist;
            double rightBackInput  = drive + strafe - twist;

            // normalize after mixing
            double maxMagnitude = Math.max(1.0,
                    Math.max(Math.abs(leftFrontInput),
                            Math.max(Math.abs(rightFrontInput),
                                    Math.max(Math.abs(leftBackInput), Math.abs(rightBackInput)))));

            lf.setPower(SpeedModifier * leftFrontInput / maxMagnitude);
            rf.setPower(SpeedModifier * rightFrontInput / maxMagnitude);
            lb.setPower(SpeedModifier * leftBackInput / maxMagnitude);
            rb.setPower(SpeedModifier * rightBackInput / maxMagnitude);

        } else {
            // Field-oriented
            double botHeading = getHeading();

            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);

            double frontLeftPower  = rotY + rotX + twist;
            double backLeftPower   = rotY - rotX + twist;
            double frontRightPower = rotY - rotX - twist;
            double backRightPower  = rotY + rotX - twist;

            // normalize after mixing
            double maxMagnitude = Math.max(1.0,
                    Math.max(Math.abs(frontLeftPower),
                            Math.max(Math.abs(backLeftPower),
                                    Math.max(Math.abs(frontRightPower), Math.abs(backRightPower)))));

            lf.setPower(SpeedModifier * frontLeftPower / maxMagnitude);
            lb.setPower(SpeedModifier * backLeftPower / maxMagnitude);
            rf.setPower(SpeedModifier * frontRightPower / maxMagnitude);
            rb.setPower(SpeedModifier * backRightPower / maxMagnitude);
        }

        // Telemetry for debugging
        TelemetryUtils.addData("Heading", getHeading());
        TelemetryUtils.addData("TargetHeading", targetHeading);
        TelemetryUtils.addData("PID Twist", twist);

        if (gamepad.back) { // Reset heading
            resetHeading();
            targetHeading = 0.0;
            headingPID.reset();
        }
    }

    public void updateTelemetry() {
        TelemetryUtils.update();
    }

    public static double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}
