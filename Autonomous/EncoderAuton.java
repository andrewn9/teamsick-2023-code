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
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// For Math.PI
import java.lang.Math;

@Autonomous(name="EncodedAuton")
public class EncoderAuton extends LinearOpMode{
    // Hardware
        // Front/back left/right mecanum wheel motors
        DcMotor fL, fR, bR, bL;

	// Array containing references to fL, fR, bR, and bL in that order
	DcMotor driveMotors[] = new DcMotor[4];

        DcMotor arm;
        Servo gripper;
	IMU imu;

    // Directions to move in
    // north means forward, south means backward, etc.
    enum MoveDir
    {
	north(0),
	east(1),
	south(2),
	west(3),
	northwest(4),
	northeast(5),
	southwest(6),
	southeast(7);
	
	// First level index in moveDirTable to the array containing the signs for each
	public final byte i;

	private MoveDir(byte i){ this.i = i; }
    }

    // Table of signs of motor powers used to move in different directions
    // To access the signs for a direction, use moveDirTable[MoveDir.<direction>.i][n] where n is in the range [0, 3]
    // n=0 accesses fl, n=1 accesses fr, n=2 accesses bl, n=3 accesses br
    // 1 = motor moves forward
    // -1 = motor moves backwards
    // 0 = motor doesn't move
    final byte moveDirTable[][] = {
        // north
        {1, 1, 1, 1},
        // east
        {1, -1, -1, 1},
        // south
        {-1, -1, -1, -1},
        // west
        {-1, 1, 1, -1},
        // northwest
        {0, 1, 0, 1},
        // northeast
        {1, 0, 0, 1},
        // southwest
        {-1, 0, 0, -1},
        // southeast
        {0, -1, -1, 0},
    };

    // Encoder ticks per 1 revolution
    final int    TICKS_PER_REV  = 1440;

    // In inches
    final double WHEEL_DIAMETER = 4.0;

    // Encoder ticks per 1 inch traveled
    final double TICKS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER * Math.PI);

    final double DRIVE_POWER = 0.5;

    // Returns the sign of a number
    private int sign(int value)
    {
	    if (value < 0)
		    return -1;
	    if (value > 0)
		    return 1;
	    return 0;
    }

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
	for (DcMotor m : driveMotors)
	{
		// Reset encoder count
		m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

		// Instruct to run to position and then brake
		m.setMode(DcMotor.RunMode.RUN_TO_POSITION);

		// Set the power
		m.setPower(DRIVE_POWER);
	}

	// Direction index
	int di = (int) direction.i;
	
	// Set target positions based on the direction each motor should move in
	fL.setTargetPosition(encoder_ticks * moveDirTable[di][0]);
	fR.setTargetPosition(encoder_ticks * moveDirTable[di][1]);
	bL.setTargetPosition(encoder_ticks * moveDirTable[di][2]);
	bR.setTargetPosition(encoder_ticks * moveDirTable[di][3]);

	// Wait until the motors have finished moving
	while (driveMotorsAreBusy())
		idle();

	// Power off the motors
	for (DcMotor m : driveMotors)
		m.setPower(0);
    }

    // Turns the robot
    private void turn(int degrees)
    {
        double botHeading = 0;
        imu.resetYaw();

	// Sign of degrees
        // + = right turn, - = left turn
	int dsign = sign(degrees);

        while (botHeading < degree -5 * dsign)
        {
            // Output yaw
            botHeading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("Bot Heading", botHeading);
            telemetry.update();
 
            // Turn with motors
            fl.setPower(DRIVE_POWER * dsign);
            fr.setPower(DRIVE_POWER * dsign);
            bl.setPower(-DRIVE_POWER * dsign);
            br.setPower(-DRIVE_POWER * dsign);

	    // Wait for a short while before checking heading again
            sleep(50);
        }

        // Brake motors
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }
    
    @Override
    public void runOpMode()
    {
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

        // Yield program until stop is requested
        while (opModeIsActive())
		idle();
    }
}
