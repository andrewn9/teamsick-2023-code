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

// For Math.PI
import java.lang.Math;

@Autonomous(name="EncodedAuton")
public class EncoderAuton extends LinearOpMode{
    // Hardware
        // Front/back left/right mecanum wheel motors
        DcMotor fL, fR, bR, bL;
    
        DcMotor arm;
        Servo gripper;

    // TODO: comment these
    final int    TICKS_PER_REV  = 1440;
    final double WHEEL_DIAMETER = 4.0;
    final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
    final double DRIVE_POWER = 0.5;

    // Reset motor position and set run mode
    private void initMotor(DcMotor motor)
    {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPosition(0);
        motor.setPower(DRIVE_POWER);
    }
    
    @Override
    public void runOpMode()
    {
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

        // TODO: create methods to strafe given a direction and distance
        // TODO: initialize imu and create methods to turn a given # degrees
	
        // Yield program until stop is requested
        while (opModeIsActive())
            sleep(1);
    }
}
