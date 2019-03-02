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
public class Driver_Mode extends RobotHardwareClass {

    //constante
    protected final int tics_per_cm = 67;
    protected final double deadzone = 0.1;
    protected final int GLISIERA_MAX = 0;
    protected final int GLISIERA_MIN = -2000;

    //conditii
    private boolean bNoContraintsMode = false;

    @Override
    public void runOpMode()
    {
        initialise(true);

        waitForStart();

        while(opModeIsActive())
        {
            gamepad_1();
            gamepad_2();

            telemetry.addData("speed FL", MotorFL.getPower());
            telemetry.addData("speed FR", MotorFL.getPower());
            telemetry.addData("gamepad y", gamepad1.left_stick_y);
            telemetry.addData("gamepad y dreapta", gamepad1.right_stick_y);
            telemetry.update();
        }
    }

    protected void gamepad_1(){
        if ( abs(gamepad1.left_stick_x) > deadzone || abs(gamepad1.left_stick_y) > deadzone || abs(gamepad1.right_stick_x) > deadzone)
            calculateWheelsPower(-gamepad1.left_stick_y , gamepad1.left_stick_x , gamepad1.right_stick_x);
        else
            stop_walk();

        //test_glisiera();
    }

    protected void gamepad_2(){
        //Pressing the two bumpers will activate constraints.
        if(gamepad2.left_bumper && gamepad2.right_bumper){
            bNoContraintsMode = !bNoContraintsMode;
        }

        //By pressing one of the triggers, the sliding mechanism will move upwards or downwards.
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

        if (gamepad2.a) {
            ContinuousServo.setPower(0.5);
        }
        else if (gamepad2.b) {
            ContinuousServo.setPower(-0.5);
        }
        else {
            ContinuousServo.setPower(0);
        }

        if (gamepad2.x) {
            FixedServo.setPosition(0.1);
        } else if (gamepad2.y) {
            FixedServo.setPosition(0.3);
        }

        telemetry.addData("Encoder Glisiera Dreapta" , MotorGlisieraR.getCurrentPosition());
        telemetry.addData("Encoder Glisiera Stanga", MotorGlisieraL.getCurrentPosition());
        telemetry.addData("No Constraints Mode", bNoContraintsMode);
    }

    protected void test_glisiera()
    {
        if (gamepad1.a) {
            MotorGlisieraL.setPower(0.9);
            MotorGlisieraR.setPower(0.9);
        }
        else if (gamepad1.b) {
            MotorGlisieraL.setPower(-0.9);
            MotorGlisieraR.setPower(-0.9);
        }
        else if (gamepad1.x) {
            MotorGlisieraL.setPower(0.6);
            MotorGlisieraR.setPower(0.6);
        }
        else if (gamepad1.y) {
            MotorGlisieraL.setPower(-0.6);
            MotorGlisieraR.setPower(-0.6);
        }
        else {
            MotorGlisieraL.setPower(0);
            MotorGlisieraR.setPower(0);
        }

        telemetry.addData("pozitie L: ", MotorGlisieraL.getCurrentPosition());
        telemetry.addData("pozitie R: ", MotorGlisieraR.getCurrentPosition());
    }

    protected void stop_walk(){
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
    }

    protected void calculateWheelsPower ( double drive, double strafe, double rotate )
    {
        double FL = Range.clip(drive + strafe + rotate , -0.7 , 0.7);
        double FR = Range.clip(drive - strafe - rotate , -0.7 , 0.7);
        double BL = Range.clip(drive - strafe + rotate , -0.7 , 0.7);
        double BR = Range.clip(drive + strafe - rotate , -0.7 , 0.7);

        MotorFL.setPower(FL);
        MotorFR.setPower(FR);
        MotorBL.setPower(BL);
        MotorBR.setPower(BR);
    }
}
