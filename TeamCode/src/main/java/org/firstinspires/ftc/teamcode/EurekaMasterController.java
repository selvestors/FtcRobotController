package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

/** this is master HW controller class. That acts as singleton for all motors, servos and other components */
@TeleOp (name="EurekaMaster")
public class EurekaMasterController extends LinearOpMode {
//public class EurekaMasterController {

    public static final String LEFT_FRONT_MOTOR = "left_front_motor_0";
    public static final String RIGHT_FRONT_MOTOR = "right_front_motor_1";
    public static final String LEFT_BACK_MOTOR = "left_back_motor_2";
    public static final String RIGHT_BACK_MOTOR = "right_back_motor_3";

    public static final String ARM_SHOULDER_LEFT_MOTOR = "arm_shoulder_left_motor_0";
    public static final String ARM_ELBOW_MOTOR = "arm_elbow_motor_1";
    public static final String ARM_SHOULDER_RIGHT_MOTOR = "arm_shoulder_right_motor_2";
    
    public static final String ARM_WRIST_SERVO = "arm_wrist_servo_0";
    public static final String LEFT_CLAW_SERVO = "left_claw_servo_1";
    public static final String RIGHT_CLAW_SERVO = "right_claw_servo_3";



    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    // Declare Eureka arm motors and servos
    private DcMotor armShoulderLeftMotor = null;
    private DcMotor armShoulderRightMotor = null;
    private DcMotor armElbowMotor = null;
    
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;
    private Servo armWristServo = null;

    @Override
    public void runOpMode() {

        

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontMotor  = hardwareMap.get(DcMotor.class, LEFT_FRONT_MOTOR);
        rightFrontMotor = hardwareMap.get(DcMotor.class, RIGHT_FRONT_MOTOR);
        leftBackMotor  = hardwareMap.get(DcMotor.class, LEFT_BACK_MOTOR);
        rightBackMotor = hardwareMap.get(DcMotor.class, RIGHT_BACK_MOTOR);

        armShoulderLeftMotor  = hardwareMap.get(DcMotor.class, ARM_SHOULDER_LEFT_MOTOR);
        armElbowMotor = hardwareMap.get(DcMotor.class, ARM_ELBOW_MOTOR);
        armShoulderRightMotor = hardwareMap.get(DcMotor.class, ARM_SHOULDER_RIGHT_MOTOR);

        leftClawServo = hardwareMap.get(Servo.class, LEFT_CLAW_SERVO);
        rightClawServo = hardwareMap.get(Servo.class, RIGHT_CLAW_SERVO);
        armWristServo = hardwareMap.get(Servo.class, ARM_WRIST_SERVO);



        // ########################################################################################
        // !!!            IMPORTANT Motor Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to Motor forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        // Stop and reset encoders
        armShoulderLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulderRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set zero power behavior
        armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armShoulderLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armShoulderRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        // runtime.reset();
        
        //set the arm wrist to neutral position
        armWristServo.setPosition(1);

        //initial position - not rotating wheels
        leftClawServo.setPosition(0.5);
        rightClawServo.setPosition(0.5);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // Send calculated power to wheels
            leftFrontMotor.setPower(leftFrontPower);
            rightFrontMotor.setPower(rightFrontPower);
            leftBackMotor.setPower(leftBackPower);
            rightBackMotor.setPower(rightBackPower);
            
            double armExtendUp = gamepad2.y ? 1.0 : 0.0;  // Y gamepad
            double armExtendDown = gamepad2.a ? -1.0 : 0.0;  // A gamepad
            double armPivotUp = gamepad2.x ? 1.0 : 0.0;  // X gamepad
            double armPivotDown = gamepad2.b ? -1.0 : 0.0;  // B gamepad
            //roll inward - pick up the sample
            double leftClawUp = gamepad2.left_trigger; //wheel claw up left
            double rightClawUp = gamepad2.left_trigger; //wheel claw up right
            
            //roll outward - eject
            double leftClawDown = -gamepad2.right_trigger; //wheel claw down left
            double rightClawDown = -gamepad2.right_trigger; //wheel claw down right
            
            double armWristUp = gamepad2.dpad_up ? 1.0 : 0.0; //wrist up
            double armWristDown = gamepad2.dpad_down ? 0.0 : 0.0; //wrist down
            
            armShoulderLeftMotor.setPower(armPivotUp);
            armShoulderRightMotor.setPower(armPivotUp);
            
            armShoulderLeftMotor.setPower(armPivotDown);
            armShoulderRightMotor.setPower(armPivotDown);
            
            armElbowMotor.setPower(armExtendUp);
            armElbowMotor.setPower(armExtendDown);
            
            //right trigger is pressed
            if(gamepad2.right_trigger>0){
                leftClawServo.setDirection(Servo.Direction.FORWARD);
                rightClawServo.setDirection(Servo.Direction.REVERSE);

                leftClawServo.setPosition(gamepad2.right_trigger);
                rightClawServo.setPosition(gamepad2.right_trigger);
            }
            else{
                leftClawServo.setPosition(0.5);
                rightClawServo.setPosition(0.5);
            }


            
            //left trigger pressed - intake
            if(gamepad2.left_trigger>0){
                leftClawServo.setDirection(Servo.Direction.REVERSE);
                rightClawServo.setDirection(Servo.Direction.FORWARD);

                leftClawServo.setPosition(gamepad2.left_trigger);
                rightClawServo.setPosition(gamepad2.left_trigger);
            }
            else{
                leftClawServo.setPosition(0.5);
                rightClawServo.setPosition(0.5);
            }
            
            // if(gamepad1.right_trigger<=0 || gamepad1.left_trigger<=0 ){
            //     leftClawServo.setPosition(0.5);
            //     rightClawServo.setPosition(0.5);
            // }

            
            
            //armWristServo.setPosition(0.0);
            
            //if D-pad up pressed, then bend wrist down
            if(gamepad2.dpad_down){
                armWristServo.setDirection(Servo.Direction.REVERSE);
                armWristServo.setPosition(1);
            }
            //if D-pad down pressed, then bend wrist up
            else if(gamepad2.dpad_up){
                armWristServo.setDirection(Servo.Direction.FORWARD);
                armWristServo.setPosition(1);
            }

            //armWristServo.setPosition(armWristDown);
            // leftClawServo.setPosition(0.0);
            //leftClawServo.setPosition(leftClawUp);
            //rightClawServo.setPosition(leftClawUp);
            //leftClawServo.setPosition(leftClawDown);
           
  
//            rightClawServo.setPosition(0.0);
            //rightClawServo.setPosition(rightClawUp);
            //rightClawServo.setPosition(rightClawDown);

            // Show the elapsed game time and all component power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("armPivotUp and down", "%4.2f, %4.2f", armPivotUp, armPivotDown);
            telemetry.addData("armExtendUp and down", "%4.2f, %4.2f", armExtendUp, armExtendDown);
            telemetry.addData("armWristUp and down", "%4.2f, %4.2f", armWristUp, armWristDown);
            telemetry.addData("leftClaw and rightClaw", "%4.2f, %4.2f", gamepad2.left_trigger, gamepad2.right_trigger);
            telemetry.addData("servo left and right", "%4.2f, %4.2f", leftClawServo.getPosition(), rightClawServo.getPosition());

            telemetry.update();
        }
    }

    // public static void main(String[] s) {
    //     EurekaMasterController emc = new EurekaMasterController();
    //     emc.runOpMode();
    // }

}
