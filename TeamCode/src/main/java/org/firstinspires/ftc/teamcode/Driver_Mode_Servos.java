package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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
                ContinuousServo.setPower(0.5);
            }
            else if (gamepad1.b) {
                ContinuousServo.setPower(-0.5);
            }
            else {
                ContinuousServo.setPower(0);
            }

            if (gamepad1.x) {
                FixedServo.setPosition(0.1);
            } else if (gamepad1.y) {
                FixedServo.setPosition(0.3);
            }

        }
    }

    public void initialise () {
        //mapare
        ContinuousServo = hardwareMap.crservo.get("ContinuousServo");
        FixedServo = hardwareMap.servo.get("FixedServo");

        //putere initiala
        ContinuousServo.setPower(0);
        FixedServo.setPosition(0.3);
        //directii
        ContinuousServo.setDirection(CRServo.Direction.FORWARD);
        FixedServo.setDirection(Servo.Direction.FORWARD);
    }
}