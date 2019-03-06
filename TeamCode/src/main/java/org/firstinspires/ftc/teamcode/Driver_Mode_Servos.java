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
<<<<<<< HEAD
           ContinuousServo.setPower(0.8);
           if ( FixedServo.getPosition() == 0 ) {
               FixedServo.setPosition(1);
           } else
           {
               if ( FixedServo.getPosition() == 1 ) {
                   FixedServo.setPosition(0);
               }
           }
=======
            if (gamepad1.a) {
                ContinuousServo.setPower(0.9);
            }
            else if (gamepad1.b) {
                ContinuousServo.setPower(-0.9);
            }
            else {
                ContinuousServo.setPower(0);
            }

            if (gamepad1.x) {
                FixedServo.setPosition(0);
            } else if (gamepad1.y) {
                FixedServo.setPosition(0.7);
            }
>>>>>>> bdf9a695e918826f3c0436e224b6ea081ec564b3
        }
    }

    protected void initialise(){
        ContinuousServo = hardwareMap.crservo.get("ContinuousServo");
        FixedServo = hardwareMap.servo.get("FixedServo");
    }

}

