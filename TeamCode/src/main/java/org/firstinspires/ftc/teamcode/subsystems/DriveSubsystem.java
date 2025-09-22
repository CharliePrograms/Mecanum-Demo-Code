package org.firstinspires.ftc.teamcode.subsystems;
<<<<<<< Updated upstream

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.DriveConstants.FieldOriented;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.DriveConstants.reverseDirections;

//import androidx.annotation.NonNull;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
=======
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
>>>>>>> Stashed changes
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
    private static DcMotorEx lf,rf,lb,rb;
    private static PIDController headingPID;
    private static double targetHeading = 0.0; // radians

    public static Boolean reverseDirections = true;
    public static Boolean FieldOriented = true;
    public static double SpeedModifier = 1.0;
    public static double Deadband = 0.05;
    // PID coefficients (currently only for heading not position)
    public static double kP = 0.01;
    public static double kI = 0.0;
    public static double kD = 0.0005;

<<<<<<< Updated upstream
    @Configurable
    public static class DriveConstants {
        public static Boolean reverseDirections = true;
        public static Boolean FieldOriented = true;
       public static double SpeedModifier = 0.75;
    }



    public static void initialize(HardwareMap hwMap) {
        left_front_motor = hwMap.get(DcMotor.class, "left_front_motor");
        right_front_motor = hwMap.get(DcMotor.class, "right_front_motor");
        left_back_motor = hwMap.get(DcMotor.class, "left_back_motor");
        right_back_motor = hwMap.get(DcMotor.class, "right_back_motor");
        gyro = hwMap.get(IMU.class, "gyro");
        gyro.initialize(
=======
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
>>>>>>> Stashed changes
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                        )
                )
        );
<<<<<<< Updated upstream
=======
        lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // disables the default velocity control
        // this does NOT disable the encoder from counting,
        // but lets us simply send raw motor power.
        lf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

>>>>>>> Stashed changes

        if (reverseDirections) {
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            lb.setDirection(DcMotorEx.Direction.REVERSE);
        } else {
            rf.setDirection(DcMotorEx.Direction.REVERSE);
            rb.setDirection(DcMotorEx.Direction.REVERSE);
        }
    }
    public void handleControllerInput(Gamepad gamepad) {
        headingPID.setGains(kP, kI, kD);
        double rawDrive  = -gamepad.left_stick_y;   // forward/backward
        double rawStrafe =  gamepad.left_stick_x;   // left/right
        double rawTwist  = -gamepad.right_stick_x;  // rotation
        // Apply deadband
        double drive = Utils.applyDeadband(rawDrive, Deadband);
        double strafe = Utils.applyDeadband(rawStrafe, Deadband);
        double twist  = Utils.applyDeadband(rawTwist, Deadband);
        // If driver is actively rotating, update target heading
        if (Math.abs(twist) > 0.05) {
            targetHeading = getHeading();
        } else {
            // Use PID to hold heading
            twist = headingPID.output(targetHeading, getHeading());
        }

        // Normalize
        double denominator = Math.max(Math.abs(drive) + Math.abs(strafe) + Math.abs(twist), 1);
        if (!FieldOriented) {
            // robot-oriented (no gyro correction)
            double left_front_input  = SpeedModifier * (drive + strafe + twist) / denominator;
            double right_front_input = SpeedModifier * (drive - strafe - twist) / denominator;
            double left_back_input   = SpeedModifier * (drive - strafe + twist) / denominator;
            double right_back_input  = SpeedModifier * (drive + strafe - twist) / denominator;

            lf.setPower(left_front_input);
            rf.setPower(right_front_input);
            lb.setPower(left_back_input);
            rb.setPower(right_back_input);
        } else {
            // field-oriented (rotate inputs by current heading)
            double botHeading = getHeading();
            double rotX = strafe * Math.cos(-botHeading) - drive * Math.sin(-botHeading);
            double rotY = strafe * Math.sin(-botHeading) + drive * Math.cos(-botHeading);
            double frontLeftPower  = SpeedModifier * (rotY + rotX + twist) / denominator;
            double backLeftPower   = SpeedModifier * (rotY - rotX + twist) / denominator;
            double frontRightPower = SpeedModifier * (rotY - rotX - twist) / denominator;
            double backRightPower  = SpeedModifier * (rotY + rotX - twist) / denominator;

            lf.setPower(frontLeftPower);
            lb.setPower(backLeftPower);
            rf.setPower(frontRightPower);
            rb.setPower(backRightPower);
        }

        // Telemetry
        TelemetryUtils.addData("Heading", getHeading());
        TelemetryUtils.addData("TargetHeading", targetHeading);
        TelemetryUtils.addData("PID Twist", twist);

        if (gamepad.back) { // reset heading on back button
            resetHeading();
            targetHeading = 0.0;
            headingPID.reset();
        }
    }

    public void updateTelemetry() {
    }

    public static double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        imu.resetYaw();
    }
}