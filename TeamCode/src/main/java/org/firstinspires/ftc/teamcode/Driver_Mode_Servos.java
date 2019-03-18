package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "Driver_Mode_Servos", group = "Driver")

public class Driver_Mode_Servos extends LinearOpMode {

<<<<<<< HEAD
    protected CRServo ContinuousServo = null;
    protected Servo FixedServo = null;
    boolean left = false;
=======
    protected CRServo ContinuousServoL = null;
    protected CRServo ContinuousServoR = null;
    protected Servo SelectionServo = null;
>>>>>>> 95e7a2767e919f953cd7f4e26b59d86943316946

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
>>>>>>> bdf9a695e918826f3c0436e224b6ea081ec564b3
        }
    }

<<<<<<< HEAD
    protected void initialise(){
        ContinuousServo = hardwareMap.crservo.get("ContinuousServo");
        FixedServo = hardwareMap.servo.get("FixedServo");
=======
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
>>>>>>> 95e7a2767e919f953cd7f4e26b59d86943316946
    }

}

