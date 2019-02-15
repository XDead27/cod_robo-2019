package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Test", group = "Autonomous")

public final class Autonomous_Test extends Autonomous_Mode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        boolean test_servo = false;

        if (test_servo){
            servo_test();
        }

    }

    protected void servo_test (){
        servo_L.setPosition(1);
        servo_R.setPosition(-1);
        while (opModeIsActive()){
            //servo_R.setPosition(1);
            //sleep (1000);
            //servo_L.setPosition(0);
            //servo_R.setPosition(0);
            //sleep (1000);
        }
    }

}
