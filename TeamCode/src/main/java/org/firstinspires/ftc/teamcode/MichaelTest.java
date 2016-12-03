package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
/**
 * Created by owner on 10/14/2016.
 */
@TeleOp(name = "MICHAEL", group = "TeleOp")
public class MichaelTest extends OpMode {

    DcMotor frontMotor;

    @Override
    public void init(){

        frontMotor = hardwareMap.dcMotor.get("FRONT_MOTOR");

    }
    @Override
    public void loop(){

        drive();

    }
    public void drive(){

        frontMotor.setPower(gamepad1.left_stick_y);
    }
}
