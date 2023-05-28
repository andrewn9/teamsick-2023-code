/*
*   Team Sick
*   Holonomic autonomous program using encoders   
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="encoded auton")
public class EncoderAuton extends LinearOpMode{
    
    private final int    TICKS_PER_REV  =   1440;
    private final double WHEEL_DIAMETER =   4.0;
    private final double TICKS_PER_INCH =   TICKS_PER_REV / (WHEEL_DIAMETER * 3.1415);
    
    private double drivePower = 0.5;

    private DcMotor fL, fR, bR, bL;
    private DcMotor arm;
    private Servo gripper;

    // Reset motor position and set run mode
    private void initMotor(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(drivePower);
    }
    
    @Override
    public void runOpMode() {
        
        // Map hardware
        fL = hardwareMap.get(DcMotor.class, "fL");
        bL = hardwareMap.get(DcMotor.class, "bL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bR = hardwareMap.get(DcMotor.class, "bR");

        gripper = hardwareMap.get(Servo.class, "gripper");        

        // Initialize motors
        initMotor(fL);
        initMotor(bL);
        initMotor(fR);
        initMotor(bR);
        
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        // Move robot forward 1"
        fL.setTargetPosition((int) TICKS_PER_INCH);
        fR.setTargetPosition((int) TICKS_PER_INCH);
        bL.setTargetPosition((int) TICKS_PER_INCH);
        bR.setTargetPosition((int) TICKS_PER_INCH);

  // todo: create methods to strafe given a direction and distance
  // todo: initialize imu and create methods to turn a given # degrees
	
        // Yield program until stop is requested
        while (opModeIsActive()) {
            sleep(0);
        }
    }
}
