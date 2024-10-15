package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public static final String RIGHT_BACK_MOTOR = "left_back_motor_3";

    public static final String ARM_SHOULDER_MOTOR = "arm_shoulder_motor_0";
    public static final String ARM_ELBOW_MOTOR = "arm_elbow_motor_1";
    
    public static final String ARM_WRIST_SERVO = "arm_wrist_servo_0";
    public static final String LEFT_CLAW_SERVO = "left_claw_servo_1";
    public static final String RIGHT_CLAW_SERVO = "right_claw_servo_2";



    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontMotor = null;
    private DcMotor leftBackMotor = null;
    private DcMotor rightFrontMotor = null;
    private DcMotor rightBackMotor = null;

    // Declare arm motors and servos
    private DcMotor armShoulderMotor = null;
    private DcMotor armElbowMotor = null;
    
    private Servo leftClawServo = null;
    private Servo rightClawServo = null;
    private Servo armWristServo = null;
    

    private DcMotor wrist = null;
    private Servo claw = null;
    private CRServo intake = null;


    // Arm and Wrist target positions for each state
    private static final int ARM_POSITION_INIT = 300;
    private static final int ARM_POSITION_INTAKE = 450;
    private static final int ARM_POSITION_WALL_GRAB = 1100;
    private static final int ARM_POSITION_WALL_UNHOOK = 1700;
    private static final int ARM_POSITION_HOVER_HIGH = 2600;
    private static final int ARM_POSITION_CLIP_HIGH = 2100;
    private static final int ARM_POSITION_LOW_BASKET = 2500;

    
    private static final int WRIST_POSITION_INIT = 0;
    private static final int WRIST_POSITION_SAMPLE = 270;
    private static final int WRIST_POSITION_SPEC = 10;


    
    // Claw positions
    private static final double CLAW_OPEN_POSITION = 0.55;
    private static final double CLAW_CLOSED_POSITION = 0.7;

    // Enum for state machine
    private enum RobotState {
        INIT,
        INTAKE,
        WALL_GRAB,
        WALL_UNHOOK,
        HOVER_HIGH,
        CLIP_HIGH,
        LOW_BASKET,
        MANUAL
    }

    // Initial state
    private RobotState currentState = RobotState.INIT;

    // Claw toggle state
    private boolean clawOpen = true;
    private boolean lastBump = false;
    private boolean lastHook = false;
    private boolean lastGrab = false;
    
    //target position
    private int targetArm = 0;
    private int targetWrist = 0;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        armShoulderMotor= hardwareMap.get(DcMotor.class, "arm");
        armElbowMotor = hardwareMap.get(DcMotor.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");
        intake = hardwareMap.get(CRServo.class, "intake");

        // Stop and reset encoders

        armShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armElbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //Set zero power behavior
        armElbowMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armShoulderMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // State machine logic
            switch (currentState) {
                case INIT:
                    targetArm = ARM_POSITION_INIT;
                    targetWrist = WRIST_POSITION_INIT;
                    telemetry.addData("State", "INIT");
                    break;
                case INTAKE:
                    targetArm = ARM_POSITION_INTAKE;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "INTAKE");
                    break;

                case WALL_GRAB:
                    targetArm = ARM_POSITION_WALL_GRAB;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_GRAB");
                    break;

                case WALL_UNHOOK:
                    targetArm = ARM_POSITION_WALL_UNHOOK;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "WALL_UNHOOK");
                    break;

                case HOVER_HIGH:
                    targetArm = ARM_POSITION_HOVER_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "HOVER_HIGH");
                    break;
                    
                case CLIP_HIGH:
                    targetArm = ARM_POSITION_CLIP_HIGH;
                    targetWrist = WRIST_POSITION_SPEC;
                    telemetry.addData("State", "CLIP_HIGH");
                    break;
                case LOW_BASKET:
                    targetArm = ARM_POSITION_LOW_BASKET;
                    targetWrist = WRIST_POSITION_SAMPLE;
                    telemetry.addData("State", "LOW_BASKET");
                    break;
                case MANUAL:
                    telemetry.addData("State", "MANUAL");
                    break;
            }
            

            // Handle state transitions based on gamepad input
            if (gamepad1.a) {
                currentState = RobotState.INTAKE;
            } else if (gamepad1.b && !lastGrab) {
                if(currentState == RobotState.WALL_GRAB){
                    currentState = RobotState.WALL_UNHOOK;
                }else{
                    currentState = RobotState.WALL_GRAB;
                }
            } else if (gamepad1.y && !lastHook) {
                if(currentState == RobotState.HOVER_HIGH){
                    currentState = RobotState.CLIP_HIGH;
                }else{
                    currentState = RobotState.HOVER_HIGH;
                }
            } else if (gamepad1.x) { 
                currentState = RobotState.LOW_BASKET;           
            } else if (gamepad1.left_bumper) {
                currentState = RobotState.INIT;
            } else if (gamepad1.dpad_up){ //manual control
                currentState = RobotState.MANUAL;
                targetArm += 10;
            } else if (gamepad1.dpad_down){
                currentState = RobotState.MANUAL;
                targetArm -= 10;
            } else if (gamepad1.dpad_left){
                currentState = RobotState.MANUAL;
                targetWrist += 1;
            } else if (gamepad1.dpad_right){
                currentState = RobotState.MANUAL;
                targetWrist -= 1;
            }
            
            lastGrab = gamepad1.b;
            lastHook = gamepad1.y;

            // Toggle claw position when right_bumper is pressed
            if (gamepad1.right_bumper && !lastBump) {
                clawOpen = !clawOpen;
                if (clawOpen) {
                    claw.setPosition(CLAW_OPEN_POSITION);
                } else {
                    claw.setPosition(CLAW_CLOSED_POSITION);
                }
            }
            lastBump = gamepad1.right_bumper;

            // Control intake servo with triggers
            if (gamepad1.right_trigger>0.1) {
                intake.setPower(1.0);
            } else if (gamepad1.left_trigger>0.1) {
                intake.setPower(-1.0);
            } else {
                intake.setPower(0);
            }
            
            
            armShoulderMotor.setTargetPosition(targetArm);
            armShoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armElbowMotor.setTargetPosition(targetWrist);
            armElbowMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armShoulderMotor.setPower(1);
            armElbowMotor.setPower(1);

            // Send telemetry data to the driver station
            telemetry.addData("Claw Position", clawOpen ? "Open" : "Closed");
            telemetry.addData("Arm Position", arm.getCurrentPosition());
            telemetry.addData("Arm Power", arm.getPower());
            telemetry.addData("Wrist Position", armElbowMotor.getCurrentPosition());
            telemetry.addData("Wrist Power", armElbowMotor.getPower());
            telemetry.update();
        }
    }
}