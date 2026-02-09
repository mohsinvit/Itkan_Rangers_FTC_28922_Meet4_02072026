package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.List;

@TeleOp(name="LimelightCode", group="Linear Opmode")
public class LimelightCode extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx shooterRight;
    private DcMotorEx shooterLeft;
    private DcMotor intake;
    private DcMotor transfer;
    private Servo blocker;

    // ---- LIMELIGHT 3A (FTC SDK) ----
    private Limelight3A limelight;

    // Set this to your AprilTag pipeline index on the Limelight
    private static final int APRILTAG_PIPELINE_INDEX = 8;

    // ---- DISTANCE CALCULATION CONSTANTS ----
    // IMPORTANT: MEASURE THESE VALUES FOR YOUR ROBOT!
    private static final double TAG_HEIGHT_INCHES = 30.0;      // Height of AprilTag center from ground
    private static final double CAMERA_HEIGHT_INCHES = 15.0;   // Height of camera lens from ground
    private static final double CAMERA_ANGLE_DEGREES = 0.0;    // Camera mounting angle (positive = angled up)

    double closed = 0.65;  // FIXED: Was 0.5
    double open = 0.45;    // FIXED: Was 0.1

    @Override
    public void runOpMode() {
        // Initialize hardware
        initHardware();

        waitForStart();

        while (opModeIsActive()) {

            // These are used later in your shooter/aim logic
            double x = 0;         // left/right offset (tx in degrees)
            double angle = 0;     // degrees (using tx)
            double distance = 0;  // distance calculated from ty
            int id = 0;

            // ---- LIMELIGHT APRILTAG DETECTION (2D MODE) ----
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
                            double ta = result.getTa();  // Target area (0-100% of image)

                            // Calculate distance using ty (vertical angle)
                            distance = calculateDistanceFromTy(ty);

                            x = tx;      // horizontal offset in degrees
                            angle = tx;  // same as x for 2D mode
                            
                            telemetry.addData("tx", tx);
                            telemetry.addData("ty", ty);
                            telemetry.addData("ta", ta);

                        } else {
                            telemetry.addData("Limelight", "Tags seen, but not ID 20/24");
                        }

                    } else {
                        telemetry.addData("Limelight", "Valid result, no tags");
                    }
                } else {
                    telemetry.addData("Limelight", "No valid target / result null");
                }
            } else {
                telemetry.addData("Limelight", "Not initialized");
            }

            //telemetry.addData("x (deg)", x);
            telemetry.addData("distance calculates (in)", distance);
            telemetry.addData("angle (deg)", angle);
            telemetry.addData("target ID", id);
            telemetry.addData("Blocker Position", blocker.getPosition());

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.right_trigger > 0.1) {
                blocker.setPosition(closed);  // Keep closed during intake
                intake.setPower(1);
                transfer.setPower(1);
            } else if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-1);
                transfer.setPower(-1);
            } else {
                intake.setPower(0);
                transfer.setPower(0);
            }

            if (gamepad1.right_bumper || gamepad1.a) {
                transfer.setPower(1);
                intake.setPower(1);

                boolean aligned = false;
                double tolerance = 1.5; // degrees tolerance for alignment

                // Get the horizontal offset and apply tag-specific adjustments
                double targetX = x; // x is already set to tx (horizontal offset in degrees)
                
                if (gamepad1.a) {
                    // Apply tag-specific offsets (in degrees)
                    if (id == 20) targetX -= 3;  // Adjust left for tag 20
                    if (id == 24) targetX += 3;  // Adjust right for tag 24
                }

                // Check if aligned - same logic as original code
                if (Math.abs(targetX) > tolerance) {
                    // NOT aligned, need to turn
                    double kp = 0.03;
                    double kf = 0.05;
                    turn = kp * targetX + kf * Math.signum(targetX);
                } else {
                    // Aligned! Within tolerance
                    aligned = true;
                }

                int target = calcTarget(distance, angle);
                if (gamepad1.a) target = 1800;

                telemetry.addData("target: ", target);
                telemetry.addData("targetX: ", targetX);
                telemetry.addData("aligned: ", aligned);
                
                double currentVelo = Math.abs(shooterLeft.getVelocity());
                double veloError = Math.abs(target - currentVelo);
                telemetry.addData("currentVelo: ", currentVelo);
                telemetry.addData("veloError: ", veloError);

                // Spin up shooter
                if (currentVelo < target) {
                    shooterRight.setPower(-1);
                    shooterLeft.setPower(-1);
                } else {
                    shooterRight.setPower(0);
                    shooterLeft.setPower(0);
                }

                // SINGLE blocker control - only open when BOTH aligned AND at velocity
                if ((aligned || targetX == 0) && veloError < 50) {
                    blocker.setPosition(open);  // 0.5 - OPEN
                    telemetry.addData("Blocker Status", "OPEN - SHOOTING");
                } else {
                 //   blocker.setPosition(closed);  // 0.7 - CLOSED
                    telemetry.addData("Blocker Status", "CLOSED - waiting");
                }
            } else {
                shooterRight.setPower(0);
                shooterLeft.setPower(0);
                blocker.setPosition(closed);  // 0.7 - CLOSED
            }

            double frpower = drive + turn - strafe;
            double brpower = drive + turn + strafe;
            double flpower = drive - turn + strafe;
            double blpower = drive - turn - strafe;

            frontRight.setPower(frpower);
            backRight.setPower(brpower);
            frontLeft.setPower(flpower);
            backLeft.setPower(blpower);

            telemetry.update();
        }

        // Stop Limelight when opmode ends
        if (limelight != null) limelight.stop();
    }

    /**
     * Calculate distance using vertical angle (ty)
     * Most accurate method for 2D tracking
     * 
     * @param ty Vertical angle offset in degrees
     * @return Distance in inches
     */
    private double calculateDistanceFromTy(double ty) {
        // Height difference between tag and camera
        double heightDifference = TAG_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;
        
        // Adjust ty for camera mounting angle
        // If camera is angled up, CAMERA_ANGLE_DEGREES is positive
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

    int calcTarget(double dist, double ang) {
        // dist is now calculated from ty in inches
        if (dist < 350) {
            return 1350;
        }
        if (dist > 350) {
            return 2000;
        }
        return 1350;
    }

    private void initLimelight() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        // Select AprilTag pipeline - ensure 2D detection is enabled in this pipeline
        limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    void initHardware() {
        initLimelight();

        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterLeft  = hardwareMap.get(DcMotorEx.class, "shooterLeft");

        intake   = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");

        blocker = hardwareMap.get(Servo.class, "blocker");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        blocker.setPosition(closed);  // Now correctly sets to 0.7 (closed)
    }
}