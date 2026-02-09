package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.List;

@Autonomous(name = "Limelightautoblue", group = "Linear OpMode")
public class Limelightautoblue extends LinearOpMode{
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooterRight, shooterLeft;
    private DcMotor intake, transfer;
    private Servo blocker;
    private Limelight3A limelight;
    private IMU imu;
   
    // Set this to your AprilTag pipeline index on the Limelight
    private static final int APRILTAG_PIPELINE_INDEX = 8;
    
    // ---- DISTANCE CALCULATION CONSTANTS ----
    private static final double TAG_HEIGHT_INCHES = 30.0;      // Height of AprilTag center from ground
    private static final double CAMERA_HEIGHT_INCHES = 15.0;   // Height of camera lens from ground
    private static final double CAMERA_ANGLE_DEGREES = 0.0;    // Camera mounting angle (positive = angled up)
   
    double closed = 0.65;
    double open = 0.45;
   
    double COUNTS_PER_INCH = 2000/72;
   
   
    @Override
    public void runOpMode(){
        initHardware();
        waitForStart();
        
        if(opModeIsActive()){
            //ball 1-3
            strafe(32, 1);
            sleep(100);
            turn(-45);
            drive(-54, -1);
            shoot(1450);
            sleep(300);
            //ball 4-6
            turn(135);
            intake(64);
            sleep(100);
            drive(45, 1);
            turn(-125);
            shoot(1450);
            sleep(300);
            // //ball 7-9
            turn(45);
            drive(-33, -1);
            turn(90);
            intake(51);
            drive(50, 1);
            turn(-90);
            drive(35, 1);
            turn(-45);
            shoot(1450);
            //ball 10-12
            turn(45);
            drive(-65, -1);
            turn(90);
            intake(60);
            drive(50, 1);
            sleep(100);
            turn(-90);
            drive(54, 1);
            turn(-45);
            shoot(1450);
            strafe(-40,-1);
        }
        
        // Stop Limelight when done
        if (limelight != null) limelight.stop();
    }
   
    private void intake(int inches) {
        intake.setPower(1);
        transfer.setPower(1);
        sleep(200);
        drive(-inches, -1);
        transfer.setPower(0);
    }
     
    private void shoot(int velocity){
        ElapsedTime timer = new ElapsedTime();
       
        double x = 0;
        double angle = 0;
        double distance = 0;
        double turn = 0;
        int id = 0;
       
        while(timer.seconds() < 2){
            turn = 0;
            x = 0;
            
            // ---- LIMELIGHT APRILTAG DETECTION ----
            if (limelight != null) {
                LLResult result = limelight.getLatestResult();

                if (result != null && result.isValid()) {
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                    if (fiducials != null && !fiducials.isEmpty()) {
                        // Pick tag 20 or 24 if visible (prefer 20 first, then 24)
                        LLResultTypes.FiducialResult chosen = null;
                        for (LLResultTypes.FiducialResult f : fiducials) {
                            int fid = f.getFiducialId();
                            if (fid == 20) { chosen = f; break; }
                        }
                        if (chosen == null) {
                            for (LLResultTypes.FiducialResult f : fiducials) {
                                int fid = f.getFiducialId();
                                if (fid == 24) { chosen = f; break; }
                            }
                        }

                        if (chosen != null) {
                            id = chosen.getFiducialId();

                            // Use 2D targeting data
                            double tx = result.getTx();  // Horizontal offset in degrees
                            double ty = result.getTy();  // Vertical offset in degrees

                            // Calculate distance using ty (vertical angle)
                            distance = calculateDistanceFromTy(ty);

                            x = tx;      // horizontal offset in degrees
                            angle = tx;  // same as x for 2D mode
                        }
                    }
                }
            }
       
            transfer.setPower(1);
            intake.setPower(1);
           
            boolean aligned = true;
            double tolerance = 1;
       
            // Auto-alignment logic
            if(Math.abs(x) > tolerance) {
                double kp = 0.01;
                double kf = 0.1;
                turn = kp * x + kf * Math.signum(x);
            } else {
                aligned = true;
            }
       
            int target = velocity;
       
            double currentVelo = Math.abs(shooterLeft.getVelocity());
            if(currentVelo < target) {
                shooterRight.setPower(-1);
                shooterLeft.setPower(-1);
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
            }
           
            if((aligned || x == 0) && Math.abs(target - currentVelo) < 20) {
                blocker.setPosition(open);
            }
            
            double frpower = turn;
            double brpower = turn;
            double flpower = -turn;
            double blpower = -turn;
            
            telemetry.addData("x", x);
            telemetry.addData("distance", distance);
            telemetry.addData("currentVelo", currentVelo);
            telemetry.update();
            
            frontRight.setPower(frpower);
            backRight.setPower(brpower);
            frontLeft.setPower(flpower);
            backLeft.setPower(blpower);
        }
       
        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        transfer.setPower(0);
        shooterRight.setPower(0);
        shooterLeft.setPower(0);
        blocker.setPosition(closed);
    }
    
    /**
     * Calculate distance using vertical angle (ty)
     * 
     * @param ty Vertical angle offset in degrees
     * @return Distance in inches
     */
    private double calculateDistanceFromTy(double ty) {
        // Height difference between tag and camera
        double heightDifference = TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;
        
        // Adjust ty for camera mounting angle
        double adjustedAngle = ty + CAMERA_ANGLE_DEGREES;
        
        // Convert to radians for Math.tan()
        double angleRadians = Math.toRadians(adjustedAngle);
        
        // Avoid division by near-zero
        if (Math.abs(angleRadians) < 0.01) {
            return 200; // Return a large distance if angle is too small
        }
        
        // tan(angle) = opposite/adjacent
        // distance = heightDifference / tan(angle)
        double distance = heightDifference / Math.tan(angleRadians);
        
        // Return absolute value (distance is always positive)
        return Math.abs(distance);
    }
    
    private void turn(int angle){
        angle = -angle;
        imu.resetYaw();
        double currentAngle = getHeading();
        while (Math.abs(currentAngle - angle) > 0.5){
            currentAngle = getHeading();
            double error = Math.abs(currentAngle - angle);
            double kp = 0.02;
            double kf = Math.signum(angle) * 0.00;
            double pow = kp * error + kf;
            pow = Math.min(Math.max(pow,-0.6),0.6);
            pow = Math.max(0.1,Math.abs(pow)) * Math.signum(pow);
            frontRight.setPower(-Math.signum(angle) * pow);
            frontLeft.setPower(Math.signum(angle) * pow);
            backRight.setPower(-Math.signum(angle) * pow);
            backLeft.setPower(Math.signum(angle) * pow);
           
            telemetry.addData("current angle", currentAngle);
            telemetry.addData("target angle", angle);
            telemetry.addData("pow", pow);
            telemetry.addData("error,", error);
            telemetry.update();
        }

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
    }
    
    private void drive(double inches, double power){
        imu.resetYaw();
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       
        int fLT = (int)(inches * COUNTS_PER_INCH);
        int fRT = (int)(inches * COUNTS_PER_INCH);
        int bLT = (int)(inches * COUNTS_PER_INCH);
        int bRT = (int)(inches * COUNTS_PER_INCH);
       
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);

        double average = Math.abs(fLT - frontLeft.getCurrentPosition()) +
            Math.abs(fRT - frontRight.getCurrentPosition()) +
            Math.abs(bLT - backLeft.getCurrentPosition()) +
            Math.abs(bRT - backRight.getCurrentPosition());
       
        average = average/4;
       
        while(average > 300) {
            average = Math.abs(fLT - frontLeft.getCurrentPosition()) +
                Math.abs(fRT - frontRight.getCurrentPosition()) +
                Math.abs(bLT - backLeft.getCurrentPosition()) +
                Math.abs(bRT - backRight.getCurrentPosition());
       
            average = average/4;
           
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.update();
        }
        
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        double drift = getHeading();
        turn((int)drift);
    }
    
    private void strafe(double inches, double power){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       
        int fLT = (int)(inches * COUNTS_PER_INCH);
        int fRT = (int)(-inches * COUNTS_PER_INCH);
        int bLT = (int)(-inches * COUNTS_PER_INCH);
        int bRT = (int)(inches * COUNTS_PER_INCH);
       
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);

        double average = Math.abs(fLT - frontLeft.getCurrentPosition()) +
            Math.abs(fRT - frontRight.getCurrentPosition()) +
            Math.abs(bLT - backLeft.getCurrentPosition()) +
            Math.abs(bRT - backRight.getCurrentPosition());
       
        average = average/4;
        imu.resetYaw();
        
        while(average > 300) {
            average = Math.abs(fLT - frontLeft.getCurrentPosition()) +
                Math.abs(fRT - frontRight.getCurrentPosition()) +
                Math.abs(bLT - backLeft.getCurrentPosition()) +
                Math.abs(bRT - backRight.getCurrentPosition());
       
            average = average/4;
           
            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.update();
        }
        
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
        double drift = getHeading();
        turn((int)drift);
    }
    
    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);
        
        // Select AprilTag pipeline
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.setPollRateHz(100);
        limelight.start();
        
        // Wait for camera to initialize
        sleep(1000);
    }
    
    private double getHeading(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        return angles.getYaw(AngleUnit.DEGREES);
    }
   
    public void initHardware(){
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
       
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");
       
        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        blocker = hardwareMap.get(Servo.class, "blocker");
       
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
            )
        );
           
        imu.initialize(parameters);
        imu.resetYaw();
       
        // Motor directions
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       
        // Zero power behaviors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
       
        // Run modes
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        // Initial servo position
        blocker.setPosition(closed);
       
        // Initialize Limelight LAST after other hardware
        initLimelight();
    }
}
