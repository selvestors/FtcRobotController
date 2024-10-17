package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="RobotArmTeleOpPosition", group="Linear Opmode")
public class RobotArmTeleOpPosition extends LinearOpMode {

    // Declare OpMode members.
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
    
    private Servo armWristServo = null;
    private CRServo leftClawServo = null;
    private CRServo rightClawServo = null;

    //private Servo claw = null;
    //private CRServo intake = null;

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

    // Claw toggle state
    //private boolean clawOpen = true;
    //private boolean lastBump = false;
   // private boolean lastHook = false;
   // private boolean lastGrab = false;
    
    //target position
    private int targetArm = 0;
    private int targetWrist = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        armShoulderLeftMotor= hardwareMap.get(DcMotor.class, ARM_SHOULDER_LEFT_MOTOR);
        armShoulderRightMotor= hardwareMap.get(DcMotor.class, ARM_SHOULDER_RIGHT_MOTOR);
        armElbowMotor = hardwareMap.get(DcMotor.class, ARM_ELBOW_MOTOR);
        
        armWristServo = hardwareMap.get(Servo.class, ARM_WRIST_SERVO);
        leftClawServo = hardwareMap.get(CRServo.class, LEFT_CLAW_SERVO);
        rightClawServo = hardwareMap.get(CRServo.class, RIGHT_CLAW_SERVO);

        // Stop and reset encoders
        armShoulderLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armShoulderRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set zero power behavior
        armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armShoulderLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armShoulderRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("inside while", currentState);
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
            
           //lastGrab = gamepad2.b;
           // lastHook = gamepad2.y;

          
            
            
            
            /*armShoulderLeftMotor.setTargetPosition(targetArm);
            armShoulderLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armShoulderRightMotor.setTargetPosition(targetArm);
            armShoulderRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armElbowMotor.setTargetPosition(targetWrist);
            armElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armShoulderLeftMotor.setPower(1);
            armShoulderRightMotor.setPower(1);
            armElbowMotor.setPower(1);
            */
            initWristServo();
            initClaws();
            // Send telemetry data to the driver station
            //telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addData("Arm Shoulder Left Position ", armShoulderLeftMotor.getCurrentPosition());
            telemetry.addData("Arm Shoulder Left Power", armShoulderLeftMotor.getPower());
            telemetry.addData("Arm Shoulder Right Position ", armShoulderRightMotor.getCurrentPosition());
            telemetry.addData("Arm Shoulder Right Power", armShoulderRightMotor.getPower());
            telemetry.addData("Elbow Position", armElbowMotor.getCurrentPosition());
            telemetry.addData("Elbow Power", armElbowMotor.getPower());

            telemetry.update();
        }
        
        
    }//end of runOpMode
    
    public void initWristServo() {
        
             if(gamepad2.b) {
                telemetry.addData("Button B pressed", armWristServo.getPosition());
                armWristServo.setPosition(0.5);
                telemetry.addData("Button B pressed", armWristServo.getPosition());
            } else {
                 telemetry.addData("Button B released", armWristServo.getPosition());
                armWristServo.setPosition(0.0);
                telemetry.addData("Button B released", armWristServo.getPosition());
            }
    } //end of initWristServo()
    
    public void initClaws() {
            // Toggle claw rotation when right or left triggers are pressed
            // Control intake servo with triggers
            if (gamepad2.right_trigger > 0.0) {
                leftClawServo.setPower(-CLAW_ROTATE_CLOCKWISE);
                rightClawServo.setPower(CLAW_ROTATE_CLOCKWISE);
                
                telemetry.addData("Right Trigger pressed", leftClawServo.getPower());
                
            } else if (gamepad2.left_trigger > 0.0) {
                leftClawServo.setPower(CLAW_ROTATE_CLOCKWISE);
                rightClawServo.setPower(-CLAW_ROTATE_CLOCKWISE);
                telemetry.addData("Left Trigger pressed", rightClawServo.getPower());
                
            } else {
                leftClawServo.setPower(0.0);
                rightClawServo.setPower(0.0);
                telemetry.addData("Triggers released", rightClawServo.getPower());
            }

    }
}