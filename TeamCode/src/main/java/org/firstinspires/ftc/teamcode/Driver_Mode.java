package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;

@TeleOp (name = "Driver_Mode", group = "Driver")
public class Driver_Mode extends RobotHardwareClass {

    //constante
    protected final int tics_per_cm = 67;
    protected final double deadzone = 0.1;
    protected final int GLISIERA_MAX = 3200;
    protected final int GLISIERA_MIN = 0;
    private final int EXTINDERE_MAX_GLISIERA_MAX = 5150;
    private final int EXTINDERE_MAX_GLISIERA_MIN = 3600;
    private final int EXTINDERE_DIFERENTA = EXTINDERE_MAX_GLISIERA_MAX - EXTINDERE_MAX_GLISIERA_MIN;
    private final int EXTINDERE_MIN = 0; //TODO: gaseste valori:
    private static final double INIT_ACC_SPEED = 0.2;
    private static final double MAX_ACC_SPEED = 0.9;
    private static final double ACCELERATION_INCREMENT = 0.2;
    private static final double ENCODER_AX_POS_IDEALA = 0;

    //conditii
    private boolean bNoConstraintsMode = false;
    private boolean bAccelerationMode = false;
    private boolean bHasPressedBumpers = false;
    private boolean bHasReachedMax = false;

    private static final double delay = 10;

    //variables
    private static double AccelerationSpeed = INIT_ACC_SPEED;

    @Override
    public void runOpMode()
    {
        initialise(true);

        waitForStart();

        while(opModeIsActive())
        {
            gamepad_1();
            gamepad_2();
            telemetry.update();

            sleep((long) delay);
        }
    }

    protected void gamepad_1(){

        //Pressing the right bumper of the gamepad enables acceleration mode. Pressing the left bumper disables it.
        if(gamepad1.right_bumper){
            bAccelerationMode = true;

        }
        else if(gamepad1.left_bumper){
            bAccelerationMode = false;
        }

        //If the acceleration mode is disabled, the wheels will move in their usual way. If it is enabled, the power given to the motors will increase over time.
        if(!bAccelerationMode) {
            if (abs(gamepad1.left_stick_x) > deadzone || abs(gamepad1.left_stick_y) > deadzone || abs(gamepad1.right_stick_x) > deadzone) {
                calculateWheelsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0.7);
            }
            else
                stop_walk();
        }
        else{
            if (abs(gamepad1.left_stick_x) > deadzone || abs(gamepad1.left_stick_y) > deadzone || abs(gamepad1.right_stick_x) > deadzone) {
                AccelerationSpeed += (delay / 1000) * ACCELERATION_INCREMENT;
                AccelerationSpeed = Range.clip(AccelerationSpeed, 0, MAX_ACC_SPEED);
                calculateWheelsPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, AccelerationSpeed);
            }
            else {
                AccelerationSpeed = INIT_ACC_SPEED;
                stop_walk();
            }
        }

        if (gamepad1.dpad_up){
            while (opModeIsActive() && !gamepad1.dpad_down){
                PowerMotoareGlisiera(-0.4);
            }
        }

        //Pressing the X button moves the left blocking servo to the blocking position, whereas pressing Y moves it to the opening one
        if (gamepad1.x) {
            ServoBlocareL.setPosition(0.5);
        } else if (gamepad1.y) {
            ServoBlocareL.setPosition(1);
        }

        //Pressing the A button moves the right blocking servo to its blocking position, whereas pressing B moves it to the opening one
        if (gamepad1.a) {
            ServoBlocareR.setPosition(0.9);
        } else if (gamepad1.b) {
            ServoBlocareR.setPosition(0.1);
        }

        telemetry.addData("Acceleration mode : ", bAccelerationMode);
        telemetry.addData("Acceleration speed : ", AccelerationSpeed);
//        telemetry.addData("FL" , MotorFL.getCurrentPosition());
//        telemetry.addData("FR" , MotorFR.getCurrentPosition());
//        telemetry.addData("BL" , MotorBL.getCurrentPosition());
//        telemetry.addData("BR" , MotorBR.getCurrentPosition());
    }

    protected void gamepad_2(){

        double MosorCoefficient = (double)MotorGlisieraR.getCurrentPosition() / (double)GLISIERA_MAX;
        double MosorMax = (EXTINDERE_DIFERENTA * MosorCoefficient) + EXTINDERE_MAX_GLISIERA_MIN;
        //telemetry.addData("Mosor max : ", MosorMax);

        //Pressing the two bumpers will activate constraints. Activating constraints means that the motors will not move past a limit that we have declared. This is done in order to prevent cases where the driver would keep extending or retracting the sliders without noticing that it is actually harmful to the mechanism itself.
        if(gamepad2.left_bumper && gamepad2.right_bumper){
            bHasPressedBumpers = true;
        }
        else if(bHasPressedBumpers){
            bNoConstraintsMode = !bNoConstraintsMode;
            bHasPressedBumpers = false;
        }

        //Extend the sliders. If the 'no constraints' mode is enabled, the sliders will extend and retract whenever input is registered on the gamepad. If not enabled, the sliders will not move if the encoders have moved up until the limit chosen by us.
        if (gamepad2.right_bumper){
            MotorExtindere.setPower(bNoConstraintsMode ? -0.9 : MotorExtindere.getCurrentPosition() > EXTINDERE_MIN? -0.9 : 0);
        }
        else if(gamepad2.left_bumper){
            MotorExtindere.setPower(bNoConstraintsMode ? 0.9 : MotorExtindere.getCurrentPosition() < MosorMax ? 0.9 : 0);
        }
        else{
            MotorExtindere.setPower(0);
        }


        //By pressing one of the triggers, the sliding mechanism will move upwards or downwards.
        if(gamepad2.right_trigger > deadzone) {
            PowerMotoareGlisiera(bNoConstraintsMode ? gamepad2.right_trigger : MotorGlisieraR.getCurrentPosition() < GLISIERA_MAX? gamepad2.right_trigger : 0);
        }
        else if(gamepad2.left_trigger > deadzone){
            PowerMotoareGlisiera(bNoConstraintsMode ? -gamepad2.left_trigger : MotorGlisieraR.getCurrentPosition() > GLISIERA_MIN? -gamepad2.left_trigger : 0);
            //OpenCloseBoxes(false);
        }
        else{
            PowerMotoareGlisiera(0);
            //telemetry.addData("Ar trebui sa se opreasca", " ");
        }


        //Rotate the brushes
        if (gamepad2.a) {
            MotorRotirePerii.setPower(0.6);
        }
        else if (gamepad2.b) {
            MotorRotirePerii.setPower(-0.6);
        }
        else {
            //if(Math.abs(MotorRotirePerii.getCurrentPosition() - ENCODER_AX_POS_IDEALA) < 5) {
                MotorRotirePerii.setPower(0);
            //}
        }

        //4 cazuri gamepad2 pt sortare :
        //-> sus - cub cub
        //-> jos - bila bila
        //-> stanga - cub bila
        //-> dreapta - bila cub

        //TODO: de verificat daca e ok SortareR dupa ce e pus iar cum era inainte
        if (gamepad2.dpad_up){
            ServoSortareL.setPosition(0.5);
            ServoSortareR.setPosition(1);
            //OpenCloseBoxes(true);

        }
        else if (gamepad2.dpad_left) {
            ServoSortareL.setPosition(0.8);
            ServoSortareR.setPosition(1);
            //OpenCloseBoxes(true);
        }
        else if (gamepad2.dpad_right){
            ServoSortareL.setPosition(0.5);
            ServoSortareR.setPosition(0.7);
            //OpenCloseBoxes(true);
        }
        else if (gamepad2.dpad_down){
            ServoSortareL.setPosition(0.8);
            ServoSortareR.setPosition(0.7);
            //OpenCloseBoxes(true);
        }

//        telemetry.addData("Encoder Mosor : " , MotorExtindere.getCurrentPosition() + " din mososr max : " + MosorMax);
//        telemetry.addData("Encoder Glisiera Stanga : ", MotorGlisieraL.getCurrentPosition());
//        telemetry.addData("Encoder Ax : ", MotorRotirePerii.getCurrentPosition());
//        telemetry.addData("No Constraints Mode : ", bNoConstraintsMode);
        telemetry.addData("Servo Blocare port", ServoBlocareL.getPortNumber());
        telemetry.addData("Servo Blocare connection", ServoBlocareL.getConnectionInfo() );
        telemetry.addData("Servo Blocare pos", ServoBlocareL.getPosition());
    }

    //FUNCTIONS

    protected void stop_walk(){
        MotorFL.setPower(0);
        MotorFR.setPower(0);
        MotorBL.setPower(0);
        MotorBR.setPower(0);
    }

    protected void calculateWheelsPower ( double drive, double strafe, double rotate, double maxspeed) {
        double FLBRNormal = Math.signum(drive)*Math.pow(drive,2) + Math.signum(strafe)*Math.pow(strafe,2);
        double FRBLNormal = Math.signum(drive)*Math.pow(drive,2) - Math.signum(strafe)*Math.pow(strafe,2);

        double ScalingCoefficient;

        maxspeed = Range.clip(maxspeed, 0, 0.9);
        if(Math.max(Math.abs(FLBRNormal), Math.abs(FLBRNormal)) > deadzone) {
            ScalingCoefficient = maxspeed / Math.max(Math.abs(FLBRNormal), Math.abs(FRBLNormal));
        }
        else{
            ScalingCoefficient = 0;
        }

        double SpeedFLBR = FLBRNormal * ScalingCoefficient;
        double SpeedFRBL = FRBLNormal * ScalingCoefficient;

//        telemetry.addData("FLBR Normal : ", FLBRNormal);
//        telemetry.addData("FLBR Normal : ", FRBLNormal);
//        telemetry.addData("FLBR Speed : ", SpeedFLBR);
//        telemetry.addData("FLBR Speed : ", SpeedFRBL);
//        telemetry.addData("FLBR Final : ", SpeedFLBR + rotate);
//        telemetry.addData("FLBR Final : ", SpeedFRBL + rotate);
//        telemetry.addData("Scaling Coefficient : ", ScalingCoefficient);

        MotorFL.setPower(Range.clip(SpeedFLBR + rotate, -maxspeed, maxspeed));
        MotorFR.setPower(Range.clip(SpeedFRBL - rotate, -maxspeed, maxspeed));
        MotorBL.setPower(Range.clip(SpeedFRBL + rotate, -maxspeed, maxspeed));
        MotorBR.setPower(Range.clip(SpeedFLBR - rotate, -maxspeed, maxspeed));
    }

    //This function allows both sliders to move in the same direction.
    private void PowerMotoareGlisiera(double speed){
        speed = Range.clip(speed, -0.9, 0.9);

        MotorGlisieraR.setPower(speed);
        MotorGlisieraL.setPower(speed);
    }

    //MACROS
    private boolean GatherAndExtendMACRO(byte dir){
        PowerMotoareGlisiera(dir * 0.7);
        return false;
    }

    private void OpenCloseBoxes(boolean open){
        if(open){
            ServoBlocareL.setPosition(1);
            ServoBlocareR.setPosition(0.1);
        }
        else{
            ServoBlocareL.setPosition(0);
            ServoBlocareR.setPosition(0.9);
        }
    }
}
