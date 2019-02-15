package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Servos", group = "Driver")
public class Test_Servos extends LinearOpMode {

    protected CRServo ServoPeriiL = null;
    protected CRServo ServoPeriiR = null;
    protected Servo ServoAlegereL = null;
    protected Servo ServoAlegereR = null;

    private double deadzone = 0.1;

    @Override
    public void runOpMode() {
        ServoPeriiL = hardwareMap.crservo.get("ServoPeriiL");
        ServoPeriiR = hardwareMap.crservo.get("ServoPeriiR");
        //ServoAlegereL = hardwareMap.servo.get("ServoAlegereL");
        //ServoAlegereR = hardwareMap.servo.get("ServoAlegereR");

        ServoPeriiL.setDirection(DcMotorSimple.Direction.FORWARD);
        ServoPeriiR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){
            if(gamepad2.left_bumper){
                ServoPeriiL.setPower(0.7);
                ServoPeriiR.setPower(0.7);
            }else if(gamepad2.right_bumper){
                ServoPeriiL.setPower(-0.7);
                ServoPeriiR.setPower(-0.7);
            }else{
                ServoPeriiL.setPower(0);
                ServoPeriiR.setPower(0);
            }

            /*if(gamepad2.dpad_left){
                ServoAlegereL.setPosition(0.8);
            }else if(gamepad2.dpad_right){
                ServoAlegereL.setPosition(0.2);
            }*/

        }
    }
}
