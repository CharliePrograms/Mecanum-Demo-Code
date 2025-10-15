package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcode.hardware.Gamepad;

@TeleOp()
public class moveMotor extends OpMode {
  DcMotor motor1;
  @override
  public void init() {
    motor1 = hardwareMap.get(DcMotor.class, "motor1")
  
  }

  @override
  public void loop() {
    if (gamepad1.a) {
      motor1.setPower(1);
    } else {
      motor1.setPower(0);
    }
  
  }
}
