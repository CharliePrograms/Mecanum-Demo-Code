package org.firstinspires.ftc.teamcode.opmodes.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TestTurretSubsystem;
import org.firstinspires.ftc.teamcode.utils.TelemetryUtils;

@TeleOp(name = "mainTeleop")
public class MainTeleopMode extends OpMode {

    DriveSubsystem drive_base = new DriveSubsystem();
//    TestArmSubsystem arm = new TestArmSubsystem();
//    TestTurretSubsystem turret = new TestTurretSubsystem();


    @Override
    public void init() {
        drive_base.initialize(hardwareMap);
        TelemetryUtils.initialize(telemetry);
    }

    @Override
    public void loop() {
        drive_base.handleControllerInput(gamepad1);
        TelemetryUtils.update();
    }

}
