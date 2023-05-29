/*
 *   Team Sick
 *   Holonomic drive program using gamepad
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

@TeleOp(name="RoboCentricDrive")
public class RoboCentricDrive extends OpMode {
    // Hardware
        // Front/back left/right mecanum wheel motors
        DcMotor fL, fR, bL, bR;
    
        DcMotor arm;
        Servo gripper;
    
    // Minimum and maximum arm target positions
    final int ARM_POS_MIN = 10;
    final int ARM_POS_MAX = 1500;

    // Value that the arm's position is changed by each loop() call
    final int ARM_MOVE_FACTOR = 20;

    final double ARM_POWER = 1.0;
        
    // Maximum drive motor power
    final double DRIVE_POWER = 0.7;
    
    @Override
    public void init() {
	// Map hardware
        fL = hardwareMap.dcMotor.get("fL");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        gripper = hardwareMap.servo.get("gripper");
        arm = hardwareMap.dcMotor.get("arm");

	// Configure hardware
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        
        gripper.setPosition(0);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setTargetPosition(0);
        arm.setPower(ARM_POWER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
	// Move mecanum wheels using the gamepad1 sticks
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x*1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fL.setPower((y+x+rx)/denominator * DRIVE_POWER);
        bL.setPower((y-x+rx)/denominator * DRIVE_POWER);
        bR.setPower((y+x-rx)/denominator * DRIVE_POWER);
        fR.setPower((y-x-rx)/denominator * DRIVE_POWER);

	// Move the arm using the gamepad1 dpad
        double armPosition = arm.getCurrentPosition();
        if(gamepad1.dpad_down)
	    armPosition -= ARM_MOVE_FACTOR;
        else if(gamepad1.dpad_up)
	    armPosition += ARM_MOVE_FACTOR;
        arm.setTargetPosition(clamp(armPosition,ARM_POS_MIN,ARM_POS_MAX));
        
	// Open/close the gripper using the gamepad1 dpad
        if(gamepad1.dpad_left)
           gripper.setPosition(1);
        else if(gamepad1.dpad_right)
           gripper.setPosition(0);
        
	// Telemetry
        telemetry.addData("Arm Position", armPosition);
        telemetry.update();
    }
    
    public int clamp(int val, int min, int max)
    {
        if(val<min)
            return min;
        if(val>max)
            return max;
        return val;
    }
}
