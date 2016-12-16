package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "SHALISSACODE", group = "TeleOp")
@Disabled
public class ShalissaCode extends OpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;
public void init () {
frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");
backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");
}
public void loop () {

}
public void telemetry () {
//telemetry.addData()
}
}
