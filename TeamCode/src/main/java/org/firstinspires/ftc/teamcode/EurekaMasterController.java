//just in case the arm code doesnt work when merged with EurekaMasterController
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

/** this is Eureka master HW controller class. That acts as singleton for all motors, servos and other components */
@TeleOp (name="EurekaMaster")
public class EurekaMasterController extends LinearOpMode {


    public static final String LEFT_FRONT_MOTOR = "left_front_motor_0";
    public static final String RIGHT_FRONT_MOTOR = "right_front_motor_1";
    public static final String LEFT_BACK_MOTOR = "left_back_motor_2";
    public static final String RIGHT_BACK_MOTOR = "right_back_motor_3";

    public static final String ARM_SHOULDER_LEFT_MOTOR = "arm_shoulder_left_motor_0";
    public static final String ARM_ELBOW_MOTOR = "arm_elbow_motor_1";
    public static final String ARM_SHOULDER_RIGHT_MOTOR = "arm_shoulder_right_motor_2";
    
    public static final String ARM_WRIST_SERVO = "arm_wrist_servo_0";
    public static final String LEFT_CLAW_SERVO = "left_claw_servo_1";
    public static final String RIGHT_CLAW_SERVO = "right_claw_servo_2";



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
    
    private CRServo leftClawServo = null;
    private CRServo rightClawServo = null;
    private Servo armWristServo = null;
    // private CRServo  armWristServo = null; 

    
    // Arm and Wrist target positions for each state
/*  private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE = 450;
    //private static final int ARM_POSITION_WALL_GRAB = 1100;
    private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    private static final int ARM_POSITION_CLIP_LOW = 2100;
    private static final int ARM_POSITION_CLIP_HIGH = 2100;
    private static final int ARM_POSITION_LOW_BASKET = 2500;
    private static final int ARM_POSITION_HIGH_BASKET = 2500;
    private static final int ARM_POSITION_HOVER_HIGH = 2600;
*/
    private static final int ARM_POSITION_INIT = 0;
    private static final int ARM_POSITION_INTAKE = 1;
    private static final int ARM_POSITION_WALL_UNHOOK = 2;
    private static final int ARM_POSITION_LOW_CHAMBER = 3;
    private static final int ARM_POSITION_CLIP_HIGH = 4;
    private static final int ARM_POSITION_LOW_BASKET = 5;
    private static final int ARM_POSITION_HIGH_BASKET = 6;
    private static final int ARM_POSITION_HOVER_HIGH = 7;

    private static final int ARM_SLIDER_POSITION_INIT = 0;
    private static final int ARM_SLIDER_POSITION_INTAKE = 1;
    private static final int ARM_SLIDER_LOW_CHAMBER = 2;
    private static final int ARM_SLIDER_HIGH_CHAMBER = 3;
    private static final int ARM_SLIDER_LOW_BASKET = 4;
    private static final int ARM_SLIDER_HIGH_BASKET = 5;

    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_INTAKE = 0;
    private static final int WRIST_POSITION_45_DEGREE = 45;
    private static final int WRIST_POSITION_90_DEGREE = 90;
    
    
    // Claw rotations
    private static final double CLAW_ROTATE_CLOCKWISE = 1.0;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        CLIP_LOW,
        CLIP_HIGH,
        LOW_BASKET,
        HIGH_BASKET,
        HOVER_HIGH,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;
    
    //target position
    private int targetArm = 0;
    private int targetWrist = 0;
    
    
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

        armWristServo = hardwareMap.get(Servo.class, ARM_WRIST_SERVO);
        // armWristServo = hardwareMap.get(CRServo.class, ARM_WRIST_SERVO);
        
        leftClawServo = hardwareMap.get(CRServo.class, LEFT_CLAW_SERVO);
        rightClawServo = hardwareMap.get(CRServo.class, RIGHT_CLAW_SERVO);
        
        // armWristServo.scaleRange(0.0,2.0);


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
        //armShoulderLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armShoulderRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        

        //Set zero power behavior
        armShoulderLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armShoulderRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        // runtime.reset();
        
        //set the arm wrist to neutral position
        armWristServo.setPosition(0.0);
        
        armElbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbowMotor.setTargetPosition(0);
        armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            /*/
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
            
           
            
            //armShoulderLeftMotor.setTargetPosition(45);
            //armShoulderLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armShoulderRightMotor.setTargetPosition(45);
            //armShoulderRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armElbowMotor.setTargetPosition(45);
            //armElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armShoulderLeftMotor.setPower(0.5);
            //armShoulderRightMotor.setPower(0.5);
            //armElbowMotor.setPower(0.5);
            
            armShoulderLeftMotor.setPower(armPivotUp);
            armShoulderRightMotor.setPower(armPivotUp);
            
            armShoulderLeftMotor.setPower(armPivotDown);
            armShoulderRightMotor.setPower(armPivotDown);
            
            armElbowMotor.setPower(armExtendUp);
            armElbowMotor.setPower(armExtendDown);
            
            
             // Control intake servo with triggers
            if (gamepad2.right_trigger > 0.0) {
                leftClawServo.setPower(1.0);
                rightClawServo.setPower(-1.0);
                
            } else if (gamepad2.left_trigger > 0.0) {
                leftClawServo.setPower(-1.0);
                rightClawServo.setPower(1.0);
                
            } else {
                leftClawServo.setPower(0.0);
                rightClawServo.setPower(0.0);
            }
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
*/
            // Show the elapsed game time and all component power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
           //telemetry.addData("servo left and right", "%4.2f, %4.2f", leftClawServo.getPosition(), rightClawServo.getPosition());

            telemetry.update();

             //init all functions
            // initGamePadControls();
             initArm();
             initSlider();
             initWristServo();
             initClaws();
        }
    }//opMode

    // public static void main(String[] s) {
    //     EurekaMasterController emc = new EurekaMasterController();
    //     emc.runOpMode();
    // }


    
    public void initArm() {
        
        if (gamepad2.b){ 
            double power = 1.0;
            armShoulderRightMotor.setDirection(DcMotor.Direction.FORWARD);
            armShoulderRightMotor.setPower(power);
            
            armShoulderLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            armShoulderLeftMotor.setPower(power); 
            
            //armShoulderLeftMotor.setTargetPosition(targetArm);
            //armShoulderLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //armShoulderRightMotor.setTargetPosition(targetArm);
            //armShoulderRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            //armShoulderLeftMotor.setPower(0.3);
            //armShoulderRightMotor.setPower(0.3);
            
        } else if (gamepad2.x){
            
            double decrementStep = 0.1;
            double power = 1.0;
            long delay=100;
            
            while(power > 0){
                armShoulderRightMotor.setDirection(DcMotor.Direction.REVERSE);
                armShoulderRightMotor.setPower(power);
                armShoulderLeftMotor.setDirection(DcMotor.Direction.FORWARD);
                armShoulderLeftMotor.setPower(power);
                sleep(delay);
                
                power -= decrementStep;
            }
            
            power=0;
            
            armShoulderRightMotor.setPower(power);
            armShoulderLeftMotor.setPower(power);
            
            // armShoulderRightMotor.setDirection(DcMotor.Direction.REVERSE);
            // armShoulderRightMotor.setPower(1);
            // armShoulderLeftMotor.setDirection(DcMotor.Direction.REVERSE);
            // armShoulderLeftMotor.setPower(1);
            /*
            armShoulderLeftMotor.setTargetPosition(targetArm);
            armShoulderLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armShoulderRightMotor.setTargetPosition(targetArm);
            armShoulderRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
            armShoulderLeftMotor.setPower(0.3);
            armShoulderRightMotor.setPower(0.3);
            */
        }
        else{
            //set power to zero when no button is pressed
            armShoulderLeftMotor.setPower(0);
            armShoulderRightMotor.setPower(0);
        }
        
        //telemetry.addData("armPivotUp and down", "%4.2f, %4.2f", armPivotUp, armPivotDown);
           
            
        /*
        armShoulderLeftMotor.setTargetPosition(targetArm);
        armShoulderLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armShoulderRightMotor.setTargetPosition(targetArm);
        armShoulderRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        armShoulderLeftMotor.setPower(1);
        armShoulderRightMotor.setPower(1);
        armElbowMotor.setPower(1);
        
        telemetry.addData("Arm Shoulder Left Position ", armShoulderLeftMotor.getCurrentPosition());
        telemetry.addData("Arm Shoulder Left Power", armShoulderLeftMotor.getPower());
        telemetry.addData("Arm Shoulder Right Position ", armShoulderRightMotor.getCurrentPosition());
        telemetry.addData("Arm Shoulder Right Power", armShoulderRightMotor.getPower());
        */
    }
    
    public void initSlider() {
        
        if(gamepad2.y) {
            if(armElbowMotor.getCurrentPosition() >= 1700) {
                armElbowMotor.setPower(0.0);
                armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                armElbowMotor.setDirection(DcMotor.Direction.REVERSE);
                
                armElbowMotor.setTargetPosition(1700);
                armElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armElbowMotor.setPower(0.5);
            }
            telemetry.addData("Button Y pressed", armElbowMotor.getCurrentPosition());
            
        } else if(gamepad2.a){
             armElbowMotor.setDirection(DcMotor.Direction.FORWARD);
            
            armElbowMotor.setTargetPosition(0);
            armElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armElbowMotor.setPower(0.5);
            telemetry.addData("Button A pressed", armElbowMotor.getCurrentPosition());
            
        } else {
            armElbowMotor.setPower(0.0);
            armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        telemetry.addData("Slider Position", armElbowMotor.getCurrentPosition());
        telemetry.addData("Slider Power", armElbowMotor.getPower());
        
    }
    
    public void initWristServo() {
        //armWristServo.setPosition(0.0);
        
        if(gamepad2.dpad_up) {
            // armWristServo.setDirection(Servo.Direction.FORWARD);
            armWristServo.setPosition(1.0);
            telemetry.addData("Button B pressed", armWristServo.getPosition());
        } else if(gamepad2.dpad_down){
            // armWristServo.setDirection(Servo.Direction.REVERSE);
            armWristServo.setPosition(0.0);
            telemetry.addData("Button A pressed", armWristServo.getPosition());
        } else {
            // armWristServo.setPosition (0.0);
        }
    } //end of initWristServo()
    
    public void initClaws() {
            // Toggle claw rotation when right or left triggers are pressed
            // Control intake servo with triggers
            if (gamepad2.right_trigger > 0.0) {
                leftClawServo.setPower(CLAW_ROTATE_CLOCKWISE);
                rightClawServo.setPower(-CLAW_ROTATE_CLOCKWISE);
                
                telemetry.addData("Right Trigger pressed", leftClawServo.getPower());
                
            } else if (gamepad2.left_trigger > 0.0) {
                leftClawServo.setPower(-CLAW_ROTATE_CLOCKWISE);
                rightClawServo.setPower(CLAW_ROTATE_CLOCKWISE);
                telemetry.addData("Left Trigger pressed", rightClawServo.getPower());
                
            } else {
                leftClawServo.setPower(0.0);
                rightClawServo.setPower(0.0);
                telemetry.addData("Triggers released", rightClawServo.getPower());
            }

    }

    public void initGamePadControls() {
        telemetry.addData("inside initGamePadControls", currentState);
        // State machine logic
        switch (currentState) {
            
            case INIT:
                targetArm = ARM_POSITION_INIT;
                targetWrist = WRIST_POSITION_INIT;
                telemetry.addData("State", "INIT");
                break;
            case INTAKE:
                targetArm = ARM_POSITION_INTAKE;
                targetWrist = WRIST_POSITION_INTAKE;
                telemetry.addData("State", "INTAKE");
                break;

            case WALL_UNHOOK:
                targetArm = ARM_POSITION_WALL_UNHOOK;
                targetWrist = WRIST_POSITION_90_DEGREE;
                telemetry.addData("State", "WALL_UNHOOK");
                break;

            case CLIP_LOW:
                targetArm = ARM_POSITION_LOW_CHAMBER;
                targetWrist = WRIST_POSITION_90_DEGREE;
                telemetry.addData("State", "CLIP_LOW");
                break;
                
            case CLIP_HIGH:
                targetArm = ARM_POSITION_CLIP_HIGH;
                targetWrist = WRIST_POSITION_90_DEGREE;
                telemetry.addData("State", "CLIP_HIGH");
                break;
                
            case LOW_BASKET:
                targetArm = ARM_POSITION_LOW_BASKET;
                targetWrist = WRIST_POSITION_90_DEGREE;
                telemetry.addData("State", "LOW_BASKET");
                break;
                
            case HIGH_BASKET:
                targetArm = ARM_POSITION_HIGH_BASKET;
                targetWrist = WRIST_POSITION_90_DEGREE;
                telemetry.addData("State", "HIGH_BASKET");
                break;
                
            case HOVER_HIGH:
                targetArm = ARM_POSITION_HOVER_HIGH;
                targetWrist = WRIST_POSITION_90_DEGREE;
                telemetry.addData("State", "HOVER_HIGH");
                break;
                
            case MANUAL:
                telemetry.addData("State", "MANUAL");
                break;
        }
        

        // Handle state transitions based on gamepad input
        if (gamepad2.a) {
            currentState = RobotState.INTAKE;
        //} else if (gamepad2.b && !lastGrab) {
        } else if (gamepad2.b) {
            if(currentState == RobotState.WALL_GRAB){
                currentState = RobotState.WALL_UNHOOK;
            }else{
                currentState = RobotState.WALL_GRAB;
            }
       // } else if (gamepad2.y && !lastHook) {
        } else if (gamepad2.y) {
            if(currentState == RobotState.HOVER_HIGH){
                currentState = RobotState.CLIP_HIGH;
            }else{
                currentState = RobotState.HOVER_HIGH;
            }
        } else if (gamepad2.x) { 
            currentState = RobotState.LOW_BASKET;           
        } else if (gamepad2.left_bumper) {
            currentState = RobotState.INIT;
        } else if (gamepad2.dpad_up){ //manual control
            currentState = RobotState.MANUAL;
            targetArm += 10;
        } else if (gamepad2.dpad_down){
            currentState = RobotState.MANUAL;
            targetArm -= 10;
        } else if (gamepad2.dpad_left){
            currentState = RobotState.MANUAL;
            targetWrist += 1;
        } else if (gamepad2.dpad_right){
            currentState = RobotState.MANUAL;
            targetWrist -= 1;
        }
}

}


