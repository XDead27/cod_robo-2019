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

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        while (opModeIsActive()) {
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
<<<<<<< HEAD
                FixedServo.setPosition(FixedServo.MIN_POSITION);
=======
                FixedServo.setPosition(0);
>>>>>>> 4852f5247ed7b5698f72cdfe64f1b3fcc86e0e41
            } else if (gamepad1.y) {
                FixedServo.setPosition(0.5);
            }
        }
    }

    public void initialise () {
        //mapare
        ContinuousServo = hardwareMap.crservo.get("ContinuousServo");
        FixedServo = hardwareMap.servo.get("FixedServo");

        //putere initiala
        ContinuousServo.setPower(0);

        //pozitie initiala
        FixedServo.setPosition(FixedServo.MIN_POSITION);

        //directii
        ContinuousServo.setDirection(CRServo.Direction.FORWARD);
        FixedServo.setDirection(Servo.Direction.FORWARD);
    }
}

