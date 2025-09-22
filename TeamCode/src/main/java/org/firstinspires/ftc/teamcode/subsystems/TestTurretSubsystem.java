package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@Config
public class TestTurretSubsystem {
    private DcMotorEx turretMotor;
    private PIDController velocityPID;
    // Tunable from Dashboard
    public static double Kp = 0.001;
    public static double Ki = 0.0;
    public static double Kd = 0.0001;

    public void initialize(HardwareMap hwMap, Telemetry opModeTelemetry) {
        turretMotor = hwMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Initialize PID with current gains
        velocityPID = new PIDController(Kp, Ki, Kd);
        velocityPID.setOutputLimits(-0.5, 0.5);
    }

    public void runTurret(Gamepad gamepad2) {
        // Target velocity in encoder ticks per second
        double targetVelocity = -gamepad2.left_stick_x * 1000; // scale as needed
        double currentVelocity = turretMotor.getVelocity();
        // Update PID gains live from Dashboard
        velocityPID.setGains(Kp, Ki, Kd);
        // PID calculation
        double power = velocityPID.output(targetVelocity, currentVelocity);
        // Apply power
        turretMotor.setPower(power);
        // Telemetry
        TelemetryUtils.addData("TurretTargetVel", targetVelocity);
        TelemetryUtils.addData("TurretVelocity", currentVelocity);
        TelemetryUtils.addData("TurretPower", power);
    }
}
