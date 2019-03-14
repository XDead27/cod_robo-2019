package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Driver_Mode_Servos", group = "Driver")

public class Driver_Mode_Servos extends LinearOpMode {

    protected CRServo ContinuousServoL = null;
    protected CRServo ContinuousServoR = null;
    protected Servo SelectionServo = null;

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                ContinuousServoL.setPower(0.9);
                ContinuousServoR.setPower(-0.9);
            }
            else if (gamepad1.b) {
                ContinuousServoL.setPower(-0.9);
                ContinuousServoR.setPower(0.9);
            }
            else {
                ContinuousServoL.setPower(0);
                ContinuousServoR.setPower(0);
            }

            if (gamepad1.dpad_up){
                SelectionServo.setPosition(0.6);
            }
            else if (gamepad1.dpad_left){
                SelectionServo.setPosition(0.35);
            }
            else if (gamepad1.dpad_down){
                SelectionServo.setPosition(0);
            }
        }
    }

    public void initialise () {
        //mapare
        ContinuousServoL = hardwareMap.crservo.get("ContinuousServoL");
        ContinuousServoR = hardwareMap.crservo.get("ContinuousServoR");
        SelectionServo = hardwareMap.servo.get("SelectionServo");

        //putere initiala
        ContinuousServoL.setPower(0);
        ContinuousServoR.setPower(0);

        //pozitie initiala
        SelectionServo.setPosition(0);

        //directii
        ContinuousServoL.setDirection(CRServo.Direction.FORWARD);
        ContinuousServoR.setDirection(CRServo.Direction.FORWARD);
        SelectionServo.setDirection(Servo.Direction.FORWARD);
    }
}

