package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp (name = "Driver_Mode_Button_Wheels", group = "Driver")

public class Driver_Mode_Button_Wheels extends LinearOpMode {

    //motoare roti
    protected DcMotor Motor_FL = null;
    protected DcMotor Motor_FR = null;
    protected DcMotor Motor_BL = null;
    protected DcMotor Motor_BR = null;

    //motoare mecanisme

    //constante
    protected final int tics_per_cm = 67;
    protected final double deadzone = 0.1;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while(opModeIsActive())
        {
            gamepad_1();
        }
    }

    protected void initialise()
    {
        //hardware mapping
        Motor_FL = hardwareMap.dcMotor.get("Motor_FL");
        Motor_FR = hardwareMap.dcMotor.get("Motor_FR");
        Motor_BL = hardwareMap.dcMotor.get("Motor_BL");
        Motor_BR = hardwareMap.dcMotor.get("Motor_BR");

        //setare directii
        Motor_FL.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_FR.setDirection(DcMotorSimple.Direction.FORWARD);
        Motor_BL.setDirection(DcMotorSimple.Direction.REVERSE);
        Motor_BR.setDirection(DcMotorSimple.Direction.FORWARD);

        //initializare putere
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);

        //setare
        Motor_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motor_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    protected void gamepad_1(){
        if (gamepad1.a){
            Motor_FL.setPower(0.5);
        }
        else{
            Motor_FL.setPower(0);
        }

        if (gamepad1.b){
            Motor_FR.setPower(0.5);
        }
        else{
            Motor_FR.setPower(0);
        }

        if (gamepad1.x){
            Motor_BL.setPower(0.5);
        }
        else{
            Motor_BL.setPower(0);
        }

        if (gamepad1.y){
            Motor_BR.setPower(0.5);
        }
        else{
            Motor_BR.setPower(0);
        }
    }

    protected void gamepad_2(){

    }

    protected void stop_walk(){
        Motor_FL.setPower(0);
        Motor_FR.setPower(0);
        Motor_BL.setPower(0);
        Motor_BR.setPower(0);
    }

}