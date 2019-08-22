package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Juan Valasquez on 3/12/2018.
 * Cleaned up spacing by Nawras Rawas Qalaji
 */

public abstract class WiredcatsLinearOpMode extends LinearOpMode {

    //Instantiation

    //Drive Train
    public DcMotor FrontLeft;
    public DcMotor FrontRight;
    public DcMotor BackLeft;
    public DcMotor BackRight;

    //Intake
    public DcMotor IntakeLeft;
    public DcMotor IntakeRight;

    //Lift
    public DcMotor Lift;

    //Relic
    public DcMotor Relic;

    //Clamp
    public Servo Clamp;

    //Blocker
    public Servo Blocker;

    //Rotate
    public Servo Dump;

    //Alignment Arms
    public Servo AlignLeft;
    public Servo AlignRight;

    //Jewel Arms
    public Servo JewelDeploy;
    public Servo JewelHit;

    //Alignment Sensor
    public DistanceSensor DistanceLeft;
    public DistanceSensor DistanceRight;

    //Claw Sensors
    public DistanceSensor DistanceFront;
    public DistanceSensor DistanceBack;

    //Jewel Sensor
    public ColorSensor ColorJewel;

    //Timer
    public ElapsedTime Runtime = new ElapsedTime();

    //UltraSonics
    public ModernRoboticsI2cRangeSensor UltraRight;
    public ModernRoboticsI2cRangeSensor UltraLeft;
    public ModernRoboticsI2cRangeSensor UltraBack;

    //Touch Sensor
    public DigitalChannel TouchLift;

    //Imu
    public BNO055IMU Imu;
    public Orientation lastAngles = new Orientation();

    //Vuforia
    public static final String TAG = "Vuforia VuMark Sample";
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer Vuforia;
    public VuforiaTrackables relicTrackables = this.Vuforia.loadTrackablesFromAsset("RelicVuMark");

    //Controller 1 Inputs
    //Analog Sticks
    public float lefty = gamepad1.left_stick_y;
    public float leftx = gamepad1.left_stick_x;
    public float righty = gamepad1.right_stick_y;
    public float rightx = gamepad1.right_stick_x;

    //Triggers
    public float leftt = gamepad1.left_trigger;
    public float rightt = gamepad1.right_trigger;

    //Buttons
    public boolean a = gamepad1.a;
    public boolean b = gamepad1.b;
    public boolean x = gamepad1.x;
    public boolean y = gamepad1.y;

    public boolean lb = gamepad1.left_bumper;
    public boolean rb = gamepad1.right_bumper;

    public boolean up = gamepad1.dpad_up;
    public boolean down = gamepad1.dpad_down;
    public boolean left = gamepad1.dpad_left;
    public boolean right = gamepad1.dpad_right;

    //Controller 2 Inputs
    //Analog Sticks
    public float lefty2 = gamepad2.left_stick_y;
    public float leftx2 = gamepad2.left_stick_x;
    public float righty2 = gamepad2.right_stick_y;
    public float rightx2 = gamepad2.right_stick_x;

    //Triggers
    public float leftt2 = gamepad2.left_trigger;
    public float rightt2 = gamepad2.right_trigger;

    //Buttons
    public boolean a2 = gamepad2.a;
    public boolean b2 = gamepad2.b;
    public boolean x2 = gamepad2.x;
    public boolean y2 = gamepad2.y;

    public boolean lb2 = gamepad2.left_bumper;
    public boolean rb2 = gamepad2.right_bumper;

    public boolean up2 = gamepad2.dpad_up;
    public boolean down2 = gamepad2.dpad_down;
    public boolean left2 = gamepad2.dpad_left;
    public boolean right2 = gamepad2.dpad_right;


    //Values
    public double ClampInit = 0.0;
    public double BlockerInit = 1.0;
    public double DumpInit = 0.0;
    public double DumpFinal = 1.0;
    public double AlignLeftInit = 0.0;
    public double AlignLeftFinal = 1.0;
    public double AlignRightInit = 1.0;
    public double JewelDeployInit = 0.0;
    public double JewelHitInit = 0.5;
    public double JewelHitRight = 0.25;
    public double JewelHitLeft = 0.75;
    public double JewelHitMoveClose = 0.53;
    public double detectedGlyph = 1.5;
    public double noGlyph = 0;
    
    public int newfrontLeftTarget;
    public int newfrontRightTarget;
    public int newbackLeftTarget;
    public int newbackRightTarget;

    //Variables

    public double Fl = lefty + -leftx + (rightx * 0.5);
    public double Fr = lefty + leftx + (rightx * 0.5);
    public double Bl = lefty + leftx + (rightx * 0.5);
    public double Br = lefty + -leftx + (rightx * 0.5);

    public double Flp = Range.clip(Fl, -1, 1);
    public double Frp = Range.clip(Fr, -1, 1);
    public double Blp = Range.clip(Bl, -1, 1);
    public double Brp = Range.clip(Br, -1, 1);

    public int intake = 0;
    public int outtake = 0;
    public int dump = 0;
    public int clamp = 0;
    public int blocker = 0;

    public String VisionTele;

    public int picto = 0;
    public double globalAngle = .30;

    public double P, I, D = 1;
    public double integral, derivative, previous_error, error, rcw = 0;

    static final double     COUNTS_PER_MOTOR_REV    = 134.4 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

    public void inititalize(){

        //Drive Train Initialization
        FrontLeft = hardwareMap.dcMotor.get("fl");
        FrontRight = hardwareMap.dcMotor.get("fr");
        BackLeft = hardwareMap.dcMotor.get("bl");
        BackRight = hardwareMap.dcMotor.get("br");

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Intake Initialization
        IntakeLeft = hardwareMap.dcMotor.get("il");
        IntakeRight = hardwareMap.dcMotor.get("ir");

        IntakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        IntakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Lift Initialization
        Lift = hardwareMap.dcMotor.get("lift");

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Relic Initialization
        Relic = hardwareMap.dcMotor.get("Re");

        //Clamp Initialization
        Clamp = hardwareMap.servo.get("Clamp");

        Clamp.setPosition(ClampInit);

        //Blocker Initialization
        Blocker = hardwareMap.servo.get("Blocker");

        Blocker.setPosition(BlockerInit);

        //Dump Initialization
        Dump = hardwareMap.servo.get("Dump");

        Dump.setPosition(DumpInit);

        //Alignment Arms Initialization
        AlignLeft = hardwareMap.servo.get("Al");
        AlignRight = hardwareMap.servo.get("Ar");

        AlignLeft.setPosition(AlignLeftInit);
        AlignRight.setPosition(AlignRightInit);

        //Jewel Arms Initialization
        JewelDeploy = hardwareMap.servo.get("Jd");
        JewelHit = hardwareMap.servo.get("Jh");

        JewelDeploy.setPosition(JewelDeployInit);
        JewelHit.setPosition(JewelHitInit);

        //Distance Sensors Initialization
        DistanceLeft = hardwareMap.get(DistanceSensor.class, "Dl");
        DistanceRight = hardwareMap.get(DistanceSensor.class, "Dr");

        //Claw Color Sensors Initialization
        DistanceFront = hardwareMap.get(DistanceSensor.class, "Cf");
        DistanceBack = hardwareMap.get(DistanceSensor.class, "Cb");

        //Jewel Color Initialization
        ColorJewel = hardwareMap.get(ColorSensor.class, "Cj");

        //Timer Initialization
        Runtime.reset();

        //Ultrasonics Initialization
        UltraRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Ur");
        UltraLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Ul");
        UltraBack = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "Ub");

        //Touch Sensor Initialization
        TouchLift = hardwareMap.get(DigitalChannel.class, "Tl");
        TouchLift.setMode(DigitalChannel.Mode.INPUT);

        //Imu Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        Imu = hardwareMap.get(BNO055IMU.class, "imu");
        Imu.initialize(parameters);

        //Vuforia Initialization
        
        sleep(1000);
        telemetry.addData("Initialization ", "complete");
        telemetry.update();
    }

    public void outtakeByTime(double power, int time) throws InterruptedException{

        IntakeLeft.setPower(power);
        IntakeRight.setPower(power);
        sleep(time);
        IntakeLeft.setPower(0.0);
        IntakeRight.setPower(0.0);
        idle();
    }

    public void intakeByTime(double power, int time) throws InterruptedException{

        IntakeLeft.setPower(power);
        IntakeRight.setPower(power);
        sleep(time);
        IntakeLeft.setPower(0.0);
        IntakeRight.setPower(0.0);
        idle();
    }

    public void intakeByPower(double power) throws InterruptedException{

        IntakeLeft.setPower(power);
        IntakeRight.setPower(power);
        idle();
    }

    public void outtakeByPower(double power) throws InterruptedException{

        IntakeLeft.setPower(power);
        IntakeRight.setPower(power);
        idle();
    }

    public void stopIntake()throws InterruptedException{

        IntakeLeft.setPower(0.0);
        IntakeRight.setPower(0.0);
        idle();
    }

    public void jewelDetect(String color) throws InterruptedException{
        if(color == "Blue"){
            if (ColorJewel.red() > ColorJewel.blue()){

                JewelHit.setPosition(JewelHitLeft);
                sleep(500);
                JewelDeploy.setPosition(JewelDeployInit);
                JewelHit.setPosition(JewelHitInit);
                sleep(400);
                idle();
            }
            else if(ColorJewel.red() < ColorJewel.blue()){

                JewelHit.setPosition(JewelHitRight);
                sleep(500);
                JewelDeploy.setPosition(JewelDeployInit);
                JewelHit.setPosition(JewelHitInit);
                sleep(400);
                idle();
            }
            else if(ColorJewel.red() == 0 && ColorJewel.blue() == 0){

                JewelHit.setPosition(JewelHitMoveClose);
                sleep(50);
                idle();
                if (ColorJewel.red() > ColorJewel.blue()){

                    JewelHit.setPosition(JewelHitLeft);
                    sleep(500);
                    JewelDeploy.setPosition(JewelDeployInit);
                    JewelHit.setPosition(JewelHitInit);
                    sleep(400);
                    idle();
                }
                else if(ColorJewel.red() < ColorJewel.blue()){

                    JewelHit.setPosition(JewelHitRight);
                    sleep(500);
                    JewelDeploy.setPosition(JewelDeployInit);
                    JewelHit.setPosition(JewelHitInit);
                    sleep(400);
                    idle();
                }
                JewelDeploy.setPosition(JewelDeployInit);
                JewelHit.setPosition(JewelHitInit);
                sleep(400);
            }
            idle();
        }

        else if(color == "Red"){
            if (ColorJewel.red() < ColorJewel.blue()){

                JewelHit.setPosition(JewelHitLeft);
                sleep(500);
                JewelDeploy.setPosition(JewelDeployInit);
                JewelHit.setPosition(JewelHitInit);
                sleep(400);
                idle();
            }
            else if(ColorJewel.red() > ColorJewel.blue()){

                JewelHit.setPosition(JewelHitRight);
                sleep(500);
                JewelDeploy.setPosition(JewelDeployInit);
                JewelHit.setPosition(JewelHitInit);
                sleep(400);
                idle();
            }
            else if(ColorJewel.red() == 0 && ColorJewel.blue() == 0){

                JewelHit.setPosition(JewelHitMoveClose);
                sleep(50);
                idle();
                if (ColorJewel.red() < ColorJewel.blue()){

                    JewelHit.setPosition(JewelHitLeft);
                    sleep(500);
                    JewelDeploy.setPosition(JewelDeployInit);
                    JewelHit.setPosition(JewelHitInit);
                    sleep(400);
                    idle();
                }
                else if(ColorJewel.red() > ColorJewel.blue()){

                    JewelHit.setPosition(JewelHitRight);
                    sleep(500);
                    JewelDeploy.setPosition(JewelDeployInit);
                    JewelHit.setPosition(JewelHitInit);
                    sleep(400);
                    idle();
                }
                JewelDeploy.setPosition(JewelDeployInit);
                JewelHit.setPosition(JewelHitInit);
                sleep(400);
            }
            idle();
        }
    }

    public void driveToLeftWall(double power, double timeout) throws InterruptedException{
        Runtime.reset();
        
        while(opModeIsActive() && Runtime.milliseconds() < timeout && DistanceLeft.getDistance(DistanceUnit.CM) > 2){

            LRByGyro(power, timeout);
        }

        stopDrive();
        idle();
    }

    public void driveToRghtWall(double power, double timeout) throws InterruptedException{
        
        Runtime.reset();
        while(opModeIsActive() && Runtime.milliseconds() < timeout && DistanceRight.getDistance(DistanceUnit.CM) > 2){

            LRByGyro(-power, timeout);
        }

        stopDrive();
        idle();
    }

    public void intakeUntilOneGlyph(double approachSpeed, double timeout) throws InterruptedException{

        Runtime.reset();
        Runtime.startTime();
        while(opModeIsActive() && Runtime.milliseconds() < timeout && DistanceFront.getDistance(DistanceUnit.CM) < detectedGlyph){

            intakeByPower(1.0);
            FBByGyro(approachSpeed, timeout);
        }

        stopIntake();
        stopDrive();
        idle();
    }

    public void intakeUntilTwoGlyphs(double approachSpeed, double timeout) throws InterruptedException{

        Runtime.reset();
        Runtime.startTime();
        while(opModeIsActive() && Runtime.milliseconds() < timeout && DistanceFront.getDistance(DistanceUnit.CM) < detectedGlyph || DistanceBack.getDistance(DistanceUnit.CM) < detectedGlyph){

            intakeByPower(1.0);
            FBByGyro(approachSpeed, timeout);
        }

        stopIntake();
        stopDrive();
        idle();
    }

    private void resetAngle(){
        lastAngles = Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    private double getAngle(){
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = Imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public double checkDirection(){
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;
        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;
        return correction;
    }
    
    public void rotate(int degrees) {
        double  leftPower, rightPower;

        if(opModeIsActive()) {
            
            getPID(degrees, getAngle());
            if (degrees < 0) {

                leftPower = -rcw;
                rightPower = rcw;
            } else if (degrees > 0) {

                leftPower = rcw;
                rightPower = -rcw;
            } else return;

            // set power to rotate.
            FrontLeft.setPower(leftPower);
            FrontRight.setPower(rightPower);
            BackLeft.setPower(leftPower);
            BackRight.setPower(rightPower);
            
            // rotate until turn is completed.
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {
                
                }

                while (opModeIsActive() && getAngle() > degrees) {

                }
            }
            else    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {

                }

            // turn the motors off.
            FrontLeft.setPower(0.0);
            FrontRight.setPower(0.0);
            BackLeft.setPower(0.0);
            BackRight.setPower(0.0);

            // wait for rotation to stop.
            sleep(200);

            // reset angle tracking on new heading.
            resetAngle();
        }
    }

    public void Forwards(double distance, double timeout){

        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        if(opModeIsActive()){

            newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontRightTarget = FrontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newfrontLeftTarget);
            FrontRight.setTargetPosition(newfrontRightTarget);
            newbackLeftTarget = BackLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackRightTarget = BackRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            BackLeft.setTargetPosition(newbackLeftTarget);
            BackRight.setTargetPosition(newbackRightTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            getPID(((FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition()) / 4), distance);

            Runtime.reset();
            FrontLeft.setPower(rcw);
            FrontRight.setPower(rcw);
            BackLeft.setPower(rcw);
            BackRight.setPower(rcw);

            while (opModeIsActive() &&
                    (Runtime.seconds() < timeout) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition(),
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public void Backwards(double distance, double timeout){

        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        if(opModeIsActive()){

            newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontRightTarget = FrontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newfrontLeftTarget);
            FrontRight.setTargetPosition(newfrontRightTarget);
            newbackLeftTarget = BackLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackRightTarget = BackRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            BackLeft.setTargetPosition(newbackLeftTarget);
            BackRight.setTargetPosition(newbackRightTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            getPID(((FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition()) / 4), distance);

            Runtime.reset();
            FrontLeft.setPower(-rcw);
            FrontRight.setPower(-rcw);
            BackLeft.setPower(-rcw);
            BackRight.setPower(-rcw);

            while (opModeIsActive() &&
                    (Runtime.seconds() < timeout) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newfrontLeftTarget,  newfrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        FrontLeft.getCurrentPosition(),
                        FrontRight.getCurrentPosition(),
                        BackLeft.getCurrentPosition(),
                        BackRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);


            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void Left(double distance, double timeout){

        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        if(opModeIsActive()){

            newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontRightTarget = FrontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newfrontLeftTarget);
            FrontRight.setTargetPosition(newfrontRightTarget);
            newbackLeftTarget = BackLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackRightTarget = BackRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            BackLeft.setTargetPosition(newbackLeftTarget);
            BackRight.setTargetPosition(newbackRightTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            getPID(((FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition()) / 4), distance);

            Runtime.reset();
            FrontLeft.setPower(-rcw);
            FrontRight.setPower(rcw);
            BackLeft.setPower(rcw);
            BackRight.setPower(-rcw);

            while (opModeIsActive() &&
                    (Runtime.seconds() < timeout) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {
                // Display it for the driver.
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void Right(double distance, double timeout){

        if(opModeIsActive()){

            newfrontLeftTarget = FrontLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newfrontRightTarget = FrontRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            FrontLeft.setTargetPosition(newfrontLeftTarget);
            FrontRight.setTargetPosition(newfrontRightTarget);
            newbackLeftTarget = BackLeft.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            newbackRightTarget = BackRight.getCurrentPosition() + (int)(distance * COUNTS_PER_INCH);
            BackLeft.setTargetPosition(newbackLeftTarget);
            BackRight.setTargetPosition(newbackRightTarget);

            FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            getPID(((FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition()) / 4), distance);

            Runtime.reset();
            FrontLeft.setPower(rcw);
            FrontRight.setPower(-rcw);
            BackLeft.setPower(-rcw);
            BackRight.setPower(rcw);

            while (opModeIsActive() &&
                    (Runtime.seconds() < timeout) &&
                    (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy())) {

                // Display it for the driver.
            }

            // Stop all motion;
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);

            // Turn off RUN_TO_POSITION
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void FBByGyro(double power, double time) throws InterruptedException{

        Runtime.reset();
        
        double correction;
        correction = checkDirection();
        while(opModeIsActive() && time < Runtime.milliseconds()){

            FrontLeft.setPower(power + correction);
            FrontRight.setPower(power - correction);
            BackLeft.setPower(power + correction);
            BackRight.setPower(power - correction);

            idle();
        }

        stopDrive();
    }

    public void LRByGyro(double power, double time) throws InterruptedException{

        Runtime.reset();
        
        double correction;
        correction = checkDirection();

        while(opModeIsActive() && time < Runtime.milliseconds()){

            FrontLeft.setPower(-power - correction);
            FrontRight.setPower(power + correction);
            BackLeft.setPower(power + correction);
            BackRight.setPower(-power - correction);

            idle();
        }

        stopDrive();
    }

    public void getPID(double sensorValue, double desiredValue){

        resetAngle();
        
        error = (desiredValue - sensorValue);
        this.integral += (error * 0.2);
        derivative = (error - this.previous_error) / 0.2;
        this.rcw = P * error + I * this.integral + D * derivative;
    }

    public void liftByPower(double power){
        
        Lift.setPower(power);
        idle();
    }

    public void stopDrive(){

        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }

    public void resetDrive() {

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int detectPicto() {

        int Vision = 0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters VuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        VuforiaParameters.vuforiaLicenseKey = "AVPejl3/////AAAAGWSyhkBAQkDRnvXeL3i+1HFQBFmB3JIQ9LxjRIYgAGkfX0vAJcEMSoCmNmoVBti30U6L3gHSQQblRnqy4fUv2msd7a/XMUFBK3/MGEvVrUpXyGwIS0o22xE/ypsXa+bhBFkju3VwLmy8d2aj4TAIPghhOsM7ayDt37y/08OJLts08eE3UDu0rne9YM/IfnPnI3yHfk2Q3hXdb7S8Q0JP9ZCmD2n0Fjv7+hgcjE6o6I0WXWlByOVwCqqEKQpPAGBSbMV36PBekzHH4P2t2cDEro5HFfDxYFK00R40yvdL+SOxTvhrMStRK4rDID/oOr2g5V+nDl89R7JD72Taj9myrE97wZoTc0GR+/LckbbKN2N1";

        VuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.Vuforia = ClassFactory.createVuforiaLocalizer(VuforiaParameters);

        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        if (vuMark == RelicRecoveryVuMark.CENTER){
            
            Vision = 1;
            VisionTele = "Center";
            sleep(50);
            idle();
        }
        else if (vuMark == RelicRecoveryVuMark.LEFT){

            Vision = 2;
            VisionTele = "Left";
            sleep(50);
            idle();
        }
        else if (vuMark == RelicRecoveryVuMark.RIGHT){

            Vision = 3;
            VisionTele = "Right";
            sleep(50);
            idle();
        }
        sleep(50);

        Vision = picto;
        return Vision;
    }
    
    public void telemetry() throws InterruptedException{
        
        while(opModeIsActive()){

            telemetry.addData("Angle: ", getAngle());
            telemetry.addData("Error: ", checkDirection());
            telemetry.addData("PID Power: ", rcw);
            telemetry.addData("Picto Column", VisionTele);
            telemetry.addData("Encoder Targets:  ", (newfrontLeftTarget + newfrontRightTarget + newbackLeftTarget + newbackRightTarget) / 4);
            telemetry.addData("Encoder Counts: ", (FrontLeft.getCurrentPosition() + FrontRight.getCurrentPosition() + BackLeft.getCurrentPosition() + BackRight.getCurrentPosition()) / 4);
            telemetry.addData("Distance Away From Left Column: ", DistanceLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Away From Right Column: ", DistanceRight.getDistance(DistanceUnit.CM));
            telemetry.addData("Front Glyph: ", DistanceFront.getDistance(DistanceUnit.CM));
            telemetry.addData("Back Glyph: ", DistanceBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Lift Touch Sensor: ", TouchLift.getState());
            telemetry.addData("Jewel Blue: ", ColorJewel.blue());
            telemetry.addData("Jewel Red: ", ColorJewel.red());
            telemetry.addData("Runtime: ", Runtime.milliseconds());
            telemetry.addData("Back Ultra: ", UltraBack.getDistance(DistanceUnit.CM));
            telemetry.addData("Right Ultra: ", UltraRight.getDistance(DistanceUnit.CM));
            telemetry.addData("Left Ultra: ", UltraLeft.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
        idle();
    }
}
