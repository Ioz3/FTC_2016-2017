package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Created by owner on 11/3/2016.
 */
@TeleOp(name = "WheelCheckTest", group = "TeleOp")
public class WheelCheckTest extends OpMode {

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    int button = 0;

    @Override
    public void init(){

        frontRight = hardwareMap.dcMotor.get("FRONT_RIGHT");
        frontLeft  = hardwareMap.dcMotor.get("FRONT_LEFT");
        backRight  = hardwareMap.dcMotor.get("BACK_RIGHT");
        backLeft   = hardwareMap.dcMotor.get("BACK_LEFT");

    }

    @Override
    public void loop(){

        if(gamepad1.a){

            button = 1;
            telemetry.addData("FRONT_RIGHT", frontRight.getPower());
        }
        else if(gamepad1.b){

            button = 2;
            telemetry.addData("FRONT_LEFT", frontLeft.getPower());
        }
        else if(gamepad1.x){

            button = 3;
            telemetry.addData("BACK_RIGHT", backRight.getPower());
        }
        else if(gamepad1.y){

            button = 4;
            telemetry.addData("BACK_LEFT", backLeft.getPower());
        }
        else{

            button = 0;

        }

       switch(button){

           case 1:  frontRight.setPower(gamepad1.left_stick_y);
                    break;
           case 2:  frontLeft.setPower(gamepad1.left_stick_y);
                    break;
           case 3:  backRight.setPower(gamepad1.left_stick_y);
                    break;
           case 4:  backLeft.setPower(gamepad1.left_stick_y );
                    break;
           default: frontRight.setPower(0.0);
                    frontLeft.setPower(0.0);
                    backRight.setPower(0.0);
                    backLeft.setPower(0.0);

       }

    }

}
