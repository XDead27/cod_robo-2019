package org.firstinspires.ftc.teamcode;

import android.support.annotation.IntRange;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Math.abs;

@TeleOp (name = "Driver_Mode", group = "Driver")

public class Driver_Mode extends LinearOpMode {

    //motoare roti
    protected DcMotor MotorFL = null;
    protected DcMotor MotorFR = null;
    protected DcMotor MotorBL = null;
    protected DcMotor MotorBR = null;

    //motoare mecanisme
    protected DcMotor MotorGlisieraL = null;
    protected DcMotor MotorGlisieraR = null;

    //servo
    protected CRServo ServoPeriiL = null;
    protected CRServo ServoPeriiR = null;
    protected Servo ServoAlegereL = null;
    protected Servo ServoAlegereR = null;
    
    //constante
    protected final int tics_per_cm = 67;
    protected final double deadzone = 0.1;
    protected final int GLISIERA_MAX = 0;
    protected final int GLISIERA_MIN = -2000;

    //coditii
    private boolean bNoContraintsMode = false;

    @Override
    public void runOpMode()
    {
        initialise();

        waitForStart();

        while(opModeIsActive())
        {
            gamepad_1();
            gamepad_2();
        }
    }

    protected void initialise()
    {
        //hardware mapping
        MotorFL = hardwareMap.dcMotor.get("MotorFL");
        MotorFR = hardwareMap.dcMotor.get("MotorFR");
        MotorBL = hardwareMap.dcMotor.get("MotorBL");
        MotorBR = hardwareMap.dcMotor.get("MotorBR");
        MotorGlisieraL = hardwareMap.dcMotor.get("MotorGlisieraL");
        MotorGlisieraR = hardwareMap.dcMotor.get("MotorGlisieraR");
        ServoPeriiL = hardwareMap.crservo.get("ServoPeriiL");
        ServoPeriiR = hardwareMap.crservo.get("ServoPeriiR");
        ServoAlegereL = hardwareMap.servo.get("ServoAlegereL");
        ServoAlegereR = hardwareMap.servo.get("ServoAlegereR");
        
        //initializare putere
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
        MotorGlisieraL.setPower(0);
        MotorGlisieraR.setPower(0);

        //setare directii
        MotorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorGlisieraL.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorGlisieraR.setDirection(DcMotorSimple.Direction.REVERSE);

        ServoPeriiL.setDirection(DcMotorSimple.Direction.FORWARD);
        ServoPeriiR.setDirection(DcMotorSimple.Direction.FORWARD);
        
        //reset encoder
        MotorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //setare encoder
        MotorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //setare cand power == 0
        MotorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorGlisieraL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorGlisieraR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    protected void gamepad_1(){
        if ( abs(gamepad1.left_stick_x) > deadzone || abs(gamepad1.left_stick_y) > deadzone || abs(gamepad1.right_stick_x) > deadzone)
            calculateWheelsPower(gamepad1.left_stick_y , gamepad1.left_stick_x , gamepad1.right_stick_x);
        else
            stop_walk();
    }

    protected void gamepad_2(){
        /*if (gamepad2.a){
            Rotire_Glisiera_Encoder(-2000,0 , 0.5); //TODO : sa o duca la 90 de grade , viteza mare , pasi caluclati , ai grija la forward si reverse sa nu futa
        }
        else if (gamepad2.b){
            Rotire_Glisiera_Encoder(0 , 0 , 0.5); //TODO : sa o duca la 0 grade (de unde a pornit) , viteza mica , aceeasi pasi calculati dar cu minus
        }*/

        //daca se apasa cele doua bumpere se pune no constraint mode
        if(gamepad2.left_bumper && gamepad2.right_bumper){
            bNoContraintsMode = !bNoContraintsMode;
        }

        if(gamepad2.left_trigger > deadzone) {
            MotorGlisieraL.setPower(bNoContraintsMode? gamepad2.left_trigger : MotorGlisieraL.getCurrentPosition() < GLISIERA_MAX? gamepad2.left_trigger : 0);
            MotorGlisieraR.setPower(bNoContraintsMode? gamepad2.left_trigger : MotorGlisieraL.getCurrentPosition() < GLISIERA_MAX? gamepad2.left_trigger : 0);
        }else if(gamepad2.right_trigger > deadzone){
            MotorGlisieraL.setPower(bNoContraintsMode? -gamepad2.right_trigger : MotorGlisieraL.getCurrentPosition() > GLISIERA_MIN? -gamepad2.right_trigger : 0);
            MotorGlisieraR.setPower(bNoContraintsMode? -gamepad2.right_trigger : MotorGlisieraL.getCurrentPosition() > GLISIERA_MIN? -gamepad2.right_trigger : 0);
        }else{
            MotorGlisieraL.setPower(0);
            MotorGlisieraR.setPower(0);
        }

        /*if(gamepad2.left_trigger > deadzone){
            ServoPeriiL.setPower(gamepad2.left_trigger);
            ServoPeriiR.setPower(gamepad2.left_trigger);
        }else if(gamepad2.right_trigger > deadzone){
            ServoPeriiL.setPower(-gamepad2.right_trigger);
            ServoPeriiR.setPower(-gamepad2.right_trigger);
        }else{
            ServoPeriiL.setPower(0);
            ServoPeriiR.setPower(0);
        }*/

        //test_glisiera();

        //telemetry
        telemetry.addData("Encoder Glisiera Dreapta" , MotorGlisieraR.getCurrentPosition());
        telemetry.addData("Encoder Glisiera Stanga", MotorGlisieraL.getCurrentPosition());
        telemetry.addData("No Constraints Mode", bNoContraintsMode);
        telemetry.update();


    }

    protected void Rotire_Glisiera_Encoder(int pos , int pasi , double speed) {
        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (pasi == 0){
            MotorGlisieraL.setTargetPosition(pos);
            MotorGlisieraR.setTargetPosition(pos);
        }
        else{
            MotorGlisieraL.setTargetPosition(MotorGlisieraL.getCurrentPosition() + pasi);
            MotorGlisieraR.setTargetPosition(MotorGlisieraR.getCurrentPosition() + pasi);
        }

        MotorGlisieraL.setPower(speed);
        MotorGlisieraR.setPower(speed);

        while (MotorGlisieraL.isBusy() || MotorGlisieraR.isBusy() && opModeIsActive()) {
            telemetry.addData("dir L : " ,  MotorGlisieraL.getDirection());
            telemetry.addData("dir R : " ,  MotorGlisieraR.getDirection());
            telemetry.addData("encoder L", MotorGlisieraL.getCurrentPosition());
            telemetry.addData("encoder R", MotorGlisieraR.getCurrentPosition());
            telemetry.update();
            idle();
        }

        MotorGlisieraL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorGlisieraR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorGlisieraL.setPower(0);
        MotorGlisieraR.setPower(0);
    }

    protected void test_glisiera()
    {
        if (gamepad2.a) {
            MotorGlisieraL.setPower(0.9);
            MotorGlisieraR.setPower(0.9);
        }
        else if (gamepad2.b) {
            MotorGlisieraL.setPower(-0.9);
            MotorGlisieraR.setPower(-0.9);
        }
        else if (gamepad2.x) {
            MotorGlisieraL.setPower(0.6);
            MotorGlisieraR.setPower(0.6);
        }
        else if (gamepad2.y) {
            MotorGlisieraL.setPower(-0.6);
            MotorGlisieraR.setPower(-0.6);
        }
        else {
            MotorGlisieraL.setPower(0);
            MotorGlisieraR.setPower(0);
        }

        telemetry.addData("pozitie L: ", MotorGlisieraL.getCurrentPosition());
        telemetry.addData("pozitie R: ", MotorGlisieraR.getCurrentPosition());
        telemetry.update();
    }

    protected void stop_walk(){
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
    }

    protected void calculateWheelsPower ( double drive, double strafe, double rotate )
    {
        double FL = Range.clip(drive - strafe + rotate , -0.7 , 0.7);
        double FR = Range.clip(drive + strafe - rotate , -0.7 , 0.7);
        double BL = Range.clip(drive + strafe + rotate , -0.7 , 0.7);
        double BR = Range.clip(drive - strafe - rotate , -0.7 , 0.7);

        MotorFL.setPower(FL);
        MotorFR.setPower(FR);
        MotorBL.setPower(BL);
        MotorBR.setPower(BR);
    }

}