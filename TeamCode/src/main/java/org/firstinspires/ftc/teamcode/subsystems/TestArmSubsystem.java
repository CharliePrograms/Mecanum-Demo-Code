package org.firstinspires.ftc.teamcode.subsystems;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.utils.PIDController;
import org.firstinspires.ftc.robotcore.external.Telemetry;
public class TestArmSubsystem {

    private DcMotorEx armMotor;
    private PIDController armPID;

    private double targetPosition = 0;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Config
    public static class ArmConstants {
        public static double Kp = 0.01;
        public static double Ki = 0.0;
        public static double Kd = 0.001;
    }

    public void initialize(HardwareMap hwMap, Telemetry telemetry) {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotor = hwMap.get(DcMotorEx.class, "armMotor");
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // PID setup
        armPID = new PIDController(0.01,0.0,0.001);
        armPID.setOutputMax(1.0);
        armPID.reset();
    }

    public void moveArmToPosition(Gamepad gamepad2) {
        // Change target position with buttons
        if (gamepad2.a) targetPosition = 0;     // down
        if (gamepad2.b) targetPosition = 500;   // mid
        if (gamepad2.y) targetPosition = 1000;  // up

        // Current position
        double currentPosition = armMotor.getCurrentPosition();

        // Calculate PID output
        double power = armPID.output(targetPosition, currentPosition);
        armMotor.setPower(power);

        // Telemetry
        dashboardTelemetry.addData("ArmTargetPos", targetPosition);
        dashboardTelemetry.addData("ArmCurrentPos", currentPosition);
        dashboardTelemetry.addData("ArmPower", power);
        dashboardTelemetry.update();
}

}
