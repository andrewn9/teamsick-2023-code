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
    DcMotor fL, fR, bL, bR;
    DcMotor arm;
    Servo gripper;
    
    private final int armMin = 10;
    private final int armMax = 1500;
    
    private double drivePower= 0.7;
    private double armPower = 1.0;
    
    @Override
    public void init() {
        fL = hardwareMap.dcMotor.get("fL");
        bl = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");
        bR = hardwareMap.dcMotor.get("bR");
        gripper = hardwareMap.servo.get("gripper");
        
        arm = hardwareMap.dcMotor.get("arm");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(armPower);
        
        gripper.setPosition(0);
        
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x*1.1;
        double rx = gamepad1.right_stick_x;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        fL.setPower((y+x+rx)/denominator * drivePower);
        bL.setPower((y-x+rx)/denominator * drivePower);
        bR.setPower((y+x-rx)/denominator * drivePower);
        fR.setPower((y-x-rx)/denominator * drivePower);
    
        
        double armPosition = arm.getCurrentPosition();
        
        if(gamepad1.dpad_down)
        {
           arm.setTargetPosition((int)armPosition-20);
        }
        else if(gamepad1.dpad_up)
        {
           arm.setTargetPosition((int)armPosition+20);
        }
        
        arm.setTargetPosition(clamp(arm.getTargetPosition(),armMin,armMax));
        
        if(gamepad1.dpad_left)
        {
           gripper.setPosition(1);
        }
        else if(gamepad1.dpad_right)
        {
           gripper.setPosition(0);
        }
        
        telemetry.addData("Arm Position", armPosition);
        telemetry.update();
    }
    
    public int clamp(int val, int min, int max)
    {
        if(val<=min)
            return min;
        if(val>=max)
            return max;
        return val;
    }
}
