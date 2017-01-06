package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by cicada03 on 12/21/16.
 *
 * TODO: need to set a wait time for the team members auto to kick in
 * must have a delay for the team members auto to start so we can
 * sync it with our own auto
 *
 */
@Autonomous(name = "CheeseCakeAuto", group = "Auto")
public class CheeseCakeAuto extends LinearOpMode {

    //DRIVE
    DcMotor rightMotor;
    DcMotor leftMotor;

    //auto drive
    double currentTime;

    public void runOpMode(){

        roboInit();

        waitForStart();

        while(opModeIsActive()){

            goForward(0.5, 5.0);

            idle();

        }



    }

    public void roboInit(){

        rightMotor = hardwareMap.dcMotor.get("RIGHT_MOTOR");
        leftMotor  = hardwareMap.dcMotor.get("LEFT_MOTOR");


    }

    //AUTO LOOP
    public void goForward(double speed, double checkTime){

        currentTime = getRuntime();

        while(getRuntime()-currentTime< checkTime){

            rightMotor.setPower(-speed);
            leftMotor.setPower(speed);
        }

        rightMotor.setPower(0.0);
        leftMotor.setPower(0.0);

    }

    public void debug(){

        telemetry.addData("CURRENT_TIME",(getRuntime()-currentTime));
        telemetry.update();
    }
}
//yey!