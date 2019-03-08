package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "Driver_Mode_Hang", group = "driver")
@Disabled


public class Driver_Mode_Hang extends LinearOpMode {

    protected DcMotor MotorGlisieraL = null;
    protected DcMotor MotorGlisieraR = null;

    //variabile globale
    int LastPosition = 0;


    @Override
    public void runOpMode() {

        //init
        MotorGlisieraL = hardwareMap.dcMotor.get("MotorGlisieraL");
        MotorGlisieraR = hardwareMap.dcMotor.get("MotorGlisieraR");

        MotorGlisieraL.setPower(0);
        MotorGlisieraR.setPower(0);

        MotorGlisieraL.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorGlisieraR.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorGlisieraL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorGlisieraL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorGlisieraR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status :", "waiting for start");
        telemetry.update();

        //wait for start
        waitForStart();

        telemetry.addData("Status :", "running");
        telemetry.update();


        while(opModeIsActive()){

            if(gamepad1.right_trigger > 0.1){
                PowerGlisiere(gamepad1.right_trigger / 2);
            }
            else if(gamepad1.left_trigger > 0.1){
                PowerGlisiere(-gamepad1.left_trigger / 2);
            }
            else if(Math.abs(gamepad1.left_stick_y) > 0.1){
                PowerGlisiereSync(-gamepad1.left_stick_y);
            }
            else{
                StopGlisiere();
            }

        }

    }

    protected void PowerGlisiere(double speed){
        Range.clip(speed, -1, 1);

        MotorGlisieraL.setPower(speed);
        MotorGlisieraR.setPower(speed);
    }

    protected void PowerGlisiereSync(double speed){
        Range.clip(speed, -1, 1);

        double RaportDeViteze = (double)Math.min(MotorGlisieraR.getCurrentPosition(), MotorGlisieraL.getCurrentPosition()) / (double)Math.max(MotorGlisieraR.getCurrentPosition(), MotorGlisieraL.getCurrentPosition());

        RaportDeViteze = Math.abs(RaportDeViteze);

        if(MotorGlisieraR.getCurrentPosition() > MotorGlisieraL.getCurrentPosition()){
            MotorGlisieraR.setPower(RaportDeViteze * speed);
            MotorGlisieraL.setPower(speed);
        }else{
            MotorGlisieraR.setPower(speed);
            MotorGlisieraL.setPower(RaportDeViteze * speed);
        }
    }

    protected void StopGlisiere(){
        PowerGlisiere(0);
    }
}
