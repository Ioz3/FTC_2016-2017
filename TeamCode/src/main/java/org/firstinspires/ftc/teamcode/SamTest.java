package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name = "SAMCODE", group = "TeleOp")
public class SamTest extends LinearOpMode {

    DcMotor motor;
    TouchSensor touchSensor;
    ColorSensor colorSensor;

    public void roboInit () {

        motor       = hardwareMap.dcMotor.get("MOTOR");
        touchSensor = hardwareMap.touchSensor.get("sensor_touch");
        colorSensor = hardwareMap.colorSensor.get("color_sensor");
        colorSensor.enableLed(false);

    }
    public void runOpMode () {

        roboInit();

        waitForStart();

        colorSensor.enableLed(true);

        while(opModeIsActive()){

            debug();

            idle();

        }

    }

    public void debug(){

        telemetry.addData("touch_sensor_isPressed", touchSensor.isPressed());
        telemetry.addData("color", colorSensor.red());
        telemetry.update();

    }

}
