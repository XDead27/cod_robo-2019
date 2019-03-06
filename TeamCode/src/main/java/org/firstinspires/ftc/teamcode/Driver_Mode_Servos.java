package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Driver_Mode_Servos", group = "Driver")

public class Driver_Mode_Servos extends LinearOpMode {

    protected CRServo ContinuousServo = null;
    protected Servo FixedServo = null;
    boolean left = false;

    @Override
    public void runOpMode() {

        initialise();

        waitForStart();

        while (opModeIsActive()) {
           ContinuousServo.setPower(0.8);
           if ( FixedServo.getPosition() == 0 ) {
               FixedServo.setPosition(1);
           } else
           {
               if ( FixedServo.getPosition() == 1 ) {
                   FixedServo.setPosition(0);
               }
           }
        }
    }

    protected void initialise(){
        ContinuousServo = hardwareMap.crservo.get("ContinuousServo");
        FixedServo = hardwareMap.servo.get("FixedServo");
    }

}

