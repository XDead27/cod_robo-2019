package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Driver_Mode_Servos", group = "Driver")
@Disabled

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
                FixedServo.setPosition(0);
            } else if (gamepad1.y) {
                FixedServo.setPosition(0.7);
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

