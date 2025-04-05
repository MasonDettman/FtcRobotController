package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Chopper extends LinearOpMode {
    CRServo leftDrive;
    CRServo rightDrive;
    DcMotor leftPivot;
    DcMotor rightPivot;
    DcMotor thirdLeg;
    //Servo leftDoor;
    Servo rightDoor;
    Servo rightArm;
    Servo leftArm;
    CRServo belt;
    DigitalChannel closeTouch;
    DigitalChannel openTouch;

    private double lDriveSen = 0.25;
    private double lDrivePwr = 0.0;
    private double rDriveSen = 0.25;
    private double rDrivePwr = 0.0;

    public void initleftdrive(){
        leftDrive = hardwareMap.get(CRServo.class, "lDrive");
        leftDrive.setDirection(CRServo.Direction.FORWARD);
        leftDrive.setPower(lDrivePwr);
    }
    public void initrighttdrive(){
        rightDrive = hardwareMap.get(CRServo.class, "rDrive");
        rightDrive.setDirection(CRServo.Direction.FORWARD);
        rightDrive.setPower(rDrivePwr);
    }

    @Override
    public void runOpMode() {

        //Initialize Hardware variables
        leftPivot = hardwareMap.get(DcMotor.class, "lPivot");
        rightPivot = hardwareMap.get(DcMotor.class, "rPivot");
        thirdLeg = hardwareMap.get(DcMotor.class, "thirdLeg");
        //leftDoor = hardwareMap.get(Servo.class, "lDoor");
        rightDoor = hardwareMap.get(Servo.class, "rDoor");
        leftArm = hardwareMap.get(Servo.class, "lArm");
        rightArm = hardwareMap.get(Servo.class, "rArm");
        belt = hardwareMap.get(CRServo.class, "belt");
        closeTouch = hardwareMap.get(DigitalChannel.class, "closeTouch");
        openTouch = hardwareMap.get(DigitalChannel.class, "openTouch");
        initleftdrive();
        initrighttdrive();

        //Set motor directions
        thirdLeg.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Status", "Running");
            telemetry.addData("Left Drive Power", leftDrive.getPower());
            telemetry.addData("Right Drive Power", rightDrive.getPower());
            telemetry.update();


            leftDrive.setPower(gamepad1.left_stick_y * lDriveSen);
            rightDrive.setPower(-gamepad1.left_stick_y * rDriveSen);

            while(gamepad1.left_trigger > 0.05){
                leftPivot.setPower(1);
                rightPivot.setPower(1);
            }
            while(gamepad1.right_trigger > 0.05) {
                leftPivot.setPower(-1);
                rightPivot.setPower(-1);
            }
                leftPivot.setPower(0);
                rightPivot.setPower(0);
            while(gamepad1.x){
                thirdLeg.setPower(1);
            }
            while(gamepad1.y){
                thirdLeg.setPower(-1);
            }
            thirdLeg.setPower(0);


        }


    }

}
