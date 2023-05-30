/*
 *   Team Sick
 *   Holonomic autonomous program using encoders   
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

// For Math.PI
import java.lang.Math;

// For creating weak references to drive motors so they can be processed using an array (driveMotors)
import java.lang.ref.WeakReference;

// Directions to move in
// north means forward, south means backward, etc.
public enum MoveDir
{
    north(0),
    east(1),
    south(2),
    west(3),
    northwest(4),
    northeast(5),
    southwest(6),
    southeast(7)
}

// Contains motor power signs to move in a specific direction
public class MoveDirSigns
{
    // These are the motor power signs for movement
    // They can hold values of -1, 0, or 1
    // 1 means that the motor should move forward
    // -1 means that the motor should move backward
    // 0 means the motor shouldn't move
    byte fL, fR, bL, bR;

}

@Autonomous(name="EncodedAuton")
public class EncoderAuton extends LinearOpMode{
    // Hardware
        // Front/back left/right mecanum wheel motors
        DcMotor fL, fR, bR, bL;

	// Array containing references to fL, fR, bR, and bL in that order
	WeakReference<DcMotor> driveMotors[4];

        DcMotor arm;
        Servo gripper;
	IMU imu;

    // TODO: comment these
    final int    TICKS_PER_REV  = 1440;
    final double WHEEL_DIAMETER = 4.0;
    final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);
    final double DRIVE_POWER = 0.5;

    // Array of MoveDirSigns objects used to retrieve the motor signs needed to move the mecanum wheels in a specific direction
    // Should be accessed using a MoveDir direction as an index
    // ex. to get the MoveDirSigns object needed to move north, use "moveDirTable[moveDir.north]"
    MoveDirSigns moveDirTable[8];

    // Returns true if any drive motors are busy
    private boolean driveMotorsAreBusy()
    {
	    for (DcMotor m : driveMotors)
		    if (m.isBusy())
			    return true;
	    return false;
    }

    // Reset motor position and set run mode
    private void initMotor(DcMotor motor)
    {
	// Reset encoder counts kept by motors
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

	// Instruct the motors to run to the target position and then brake
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setTargetPosition(0);
        motor.setPower(DRIVE_POWER);
    }

    // Move in direction for specified encoder ticks
    private void move(MoveDir direction, int encoder_ticks)
    {
	// Reset encoder count
	// Instruct to run to position and then brake
	// Set the power
	for (DcMotor m : driveMotors)
	{
		m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		m.setPower(DRIVE_POWER);
	}
	
	// Set target positions based on the direction each motor should move in
	MoveDirSigns signs = moveDirTable[direction];
	fL.setTargetPosition(encoder_ticks * signs.fL);
	fR.setTargetPosition(encoder_ticks * signs.fR);
	bL.setTargetPosition(encoder_ticks * signs.bL);
	bR.setTargetPosition(encoder_ticks * signs.bR);

	// Wait until the motors have finished moving
	while (driveMotorsAreBusy())
		idle();

	// Power off the motors
	for (DcMotor m : driveMotors)
		m.setPower(0);
    }
    
    @Override
    public void runOpMode()
    {
	// Initialize moveDirTable
	moveDirTable[moveDir.north].fL = 1;
	moveDirTable[moveDir.north].fR = 1;
	moveDirTable[moveDir.north].bL = 1;
	moveDirTable[moveDir.north].bR = 1;

	moveDirTable[moveDir.east].fL = 1;
	moveDirTable[moveDir.east].fR = -1;
	moveDirTable[moveDir.east].bL = -1;
	moveDirTable[moveDir.east].bR = 1;

	moveDirTable[moveDir.south].fL = -1;
	moveDirTable[moveDir.south].fR = -1;
	moveDirTable[moveDir.south].bL = -1;
	moveDirTable[moveDir.south].bR = -1;

	moveDirTable[moveDir.west].fL = -1;
	moveDirTable[moveDir.west].fR = 1;
	moveDirTable[moveDir.west].bL = 1;
	moveDirTable[moveDir.west].bR = -1;

	moveDirTable[moveDir.northwest].fL = 0;
	moveDirTable[moveDir.northwest].fR = 1;
	moveDirTable[moveDir.northwest].bL = 0;
	moveDirTable[moveDir.northwest].bR = 1;

	moveDirTable[moveDir.northeast].fL = 1;
	moveDirTable[moveDir.northeast].fR = 0;
	moveDirTable[moveDir.northeast].bL = 0;
	moveDirTable[moveDir.northeast].bR = 1;

	moveDirTable[moveDir.southeast].fL = -1;
	moveDirTable[moveDir.southeast].fR = 0;
	moveDirTable[moveDir.southeast].bL = 0;
	moveDirTable[moveDir.southeast].bR = -1;

	moveDirTable[moveDir.southwest].fL = 0;
	moveDirTable[moveDir.southwest].fR = -1;
	moveDirTable[moveDir.southwest].bL = -1;
	moveDirTable[moveDir.southwest].bR = 0;
	
        // Map hardware
        fL = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        gripper = hardwareMap.get(Servo.class, "gripper");        
	imu = hardwareMap.get(IMU.class, "imu");

	// Initialize driveMotors references
	driveMotors[0] = fL;
	driveMotors[1] = fR;
	driveMotors[2] = bL;
	driveMotors[3] = bR;

        // Initialize motors
	for (DcMotor m : driveMotors)
		initMotor(m);
        fR.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.REVERSE);
	
	// Initialize IMU
	// XXX: might need to adjust initialization parameters later
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));

        waitForStart();

        // Move robot forward 1"
	move(MoveDir.north, (int) TICKS_PER_INCH);

        // TODO: initialize imu and create methods to turn a given # degrees
	
        // Yield program until stop is requested
        while (opModeIsActive())
		idle();
    }
}
