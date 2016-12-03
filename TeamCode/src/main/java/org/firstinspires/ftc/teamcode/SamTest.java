package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
public class SamTest extends OpMode {
    DcMotor motor;
    public void init () {
        motor = hardwareMap.dcMotor.get("MOTOR");
    }
    public void loop () {
        motor.setPower(1);
    }

}
