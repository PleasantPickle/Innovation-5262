package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Event;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous

public class RedRight_DeliverPreload extends LinearOpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    
    // Delcare Motors
    private DcMotor leftBack;
    private DcMotor leftFront;
    private DcMotor rightBack;
    private DcMotor rightFront;
    
    private DcMotor linearSlide;
    private DcMotor carouselSpinner;
    
    // Declare Servos and Servo Info 
    public Servo arm;
    public final static double ARM_HOME = 0.0; // Starting position for Servo Arm
    public final static double ARM_MIN_RANGE = 0.0; // Smallest number value allowed for servo poition (0 degrees)
    public final static double ARM_MAX_RANGE = 0.5; // Largest number value allowed for servo poition (90 degrees)

    // Declare motor info
    final int ANDYMARK_NEVEREST_40_RPM = 160; // Drive Train wheels
    final int ANDYMARK_NEVEREST_3_7_RPM = 1780; // carouselSpinner
    final int ANDYMARK_NEVEREST_60_RPM = 105; // linearSlide
    
    // NOTE: ALL DISTANCES DONE IN INCHES
    final double robotDiameter = 20.0; // inches
    final double wheelDiameter = 3.5; // inches
    
    
    @Override
    public void runOpMode() // throws InterruptedException
    {
        // The robot system is all configured (runOpMode configures robot system)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        
        // Initialize Motors and Servos
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        
        linearSlide = hardwareMap.get(DcMotor.class, "linearSlide");
        carouselSpinner = hardwareMap.get(DcMotor.class, "carouselSpinner");
        arm = hardwareMap.get(Servo.class, "gateArm");
        arm.setPosition(ARM_HOME);
        
        // Set zero power behavior
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carouselSpinner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        if (opModeIsActive())
        {
            try
            {
                // Move forward to team shipping hub
                drive(12, wheelDiameter, true, ANDYMARK_NEVEREST_40_RPM);
                
                // Fully Extend Linear Slide
                linearSlide.setPower(0.5);
                Thread.sleep(2700);
                linearSlide.setPower(0.0);
                
                // Open gate 
                arm.setPosition(ARM_MAX_RANGE);
                Thread.sleep(2000);
                arm.setPosition(ARM_MIN_RANGE);
                
                // Retract Linear Slide
                linearSlide.setPower(-0.5);
                Thread.sleep(2335);
                linearSlide.setPower(0.0);
                
                // Turn counter clockwise 90 degrees
                //turn(robotDiameter, wheelDiameter, 90, true, ANDYMARK_NEVEREST_40_RPM);
                // Turn clockwise 90 degrees
                
                //Thread.sleep(1000); // Wait one second
                
                //turn(robotDiameter, wheelDiameter, 90, false, ANDYMARK_NEVEREST_40_RPM);
                // Move forward 12 inches
                //drive(12, wheelDiameter, true, ANDYMARK_NEVEREST_40_RPM);
                // Move backward 12 inches
                //drive(12, wheelDiameter, false, ANDYMARK_NEVEREST_40_RPM);
            }
            catch(Exception e)
            {
            
            }
        }
    }
    
    // Methods
    public void turn(double robotDiameter, double wheelDiameter, double degreesToTurn, boolean turnCounterClockWise, int RPM)
    {
        // Calculate fractional number of times the wheel will need to make a full 360 degree rotation in order to cover the length of the robot's circumference
        double numberWheelRotationsToCoverRobotCircumference = robotDiameter / wheelDiameter;
        
        // Calculate number of times to rotate the motor for a specific number of degrees
        double numberOfRotations = numberWheelRotationsToCoverRobotCircumference * degreesToTurn / 360;
    
        // Calculate number of milliseconds that the motor will need to spin to spin a desired number of rotations
        int millis = (int)(numberOfRotations * calculateMillisPerRotation(RPM));
    
        // Set motor direction
        if (turnCounterClockWise) // rotate robot in place turning it left
        {
            // Set right side motors to spin moving the robot up
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            // Set left side motors to spin moving the robot back
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
        }
        else // rotate robot in place turning it right
        {
            // Set right side motors to spin moving the robot back
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            // Set left side motors to spin moving the robot up
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
        }
        driveForMillis(millis);
    }
    public void drive(double distance, double wheelDiameter, boolean moveUp, int RPM)
    {
        // Calculate fractional number of times the wheel will need to make a full 360 degree rotation in order to move the desired length
        double numberWheelRotationsToMoveDesiredLength = distance / wheelDiameter;
    
        // Calculate number of milliseconds that the motor will need to spin to spin a desired number of rotations
        int millis = (int)(numberWheelRotationsToMoveDesiredLength * calculateMillisPerRotation(RPM));
        
        // Set motor direction
        if (moveUp) // move robot up
        {
            // Set right side motors to spin moving the robot up
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);
            // Set left side motors to spin moving the robot up
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
        }
        else // move robot back
        {
            // Set right side motors to spin moving the robot back
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            // Set left side motors to spin moving the robot back
            leftFront.setDirection(DcMotor.Direction.REVERSE);
            leftBack.setDirection(DcMotor.Direction.REVERSE);
        }
        driveForMillis(millis);
    }
    public void driveForMillis(int millis)
    {
        // Set drive power
        setDrivingPower(0.5);
    
        try
        {
            // Wait for a specific number of milliseconds before continuing 
            Thread.sleep(millis); 
        }
        catch(Exception e)
        {
            
        }
        
    
        // Stop robot
        setDrivingPower(0.0);
    }
    public void setDrivingPower(double newDrivingPower)
    {
        rightFront.setPower(newDrivingPower);
        rightBack.setPower(newDrivingPower);
        leftFront.setPower(newDrivingPower);
        leftBack.setPower(newDrivingPower);
    }
    public int calculateMillisPerRotation(int RPM)
    {
        // Calculate the number of milliseconds per one full rotation of the motor
        return (60000/RPM); // 60000 Milliseconds Per 1 Rotation
    }
}
