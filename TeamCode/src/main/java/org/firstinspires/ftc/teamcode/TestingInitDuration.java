package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="TestingInitDuration", group ="Driver")
public abstract class TestingInitDuration extends LinearOpMode {

    protected DcMotor motor = null;

    @Override
    public void runOpMode() {
        initialise();

        //waitForStart();

        while (opModeIsActive()) {
            motor.setPower(0.3);
        }
    }

    protected void initialise() {
        motor = hardwareMap.dcMotor.get("motor");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setPower(0.7);

        while(!isStarted()){
            telemetry.addData("Inca are putere motorul", motor.getPower());
            telemetry.addData("Thread is alive? ", Thread.currentThread().isAlive());
            telemetry.addData("Thread is interrupted? ", Thread.currentThread().isInterrupted());
            telemetry.update();
        }
    }
}
