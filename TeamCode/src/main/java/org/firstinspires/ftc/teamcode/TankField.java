package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.AxesSigns;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

@TeleOp(name = "TankField")
public class TankField extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    public BNO055IMU imu;

    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    public double turn = 0;
    public double move = 0;

    public double offSetAngle = 0;

    public int target = 0;

    public double angle;

    public void setup() {
        init();
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");


        frontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        backLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        backRight = hardwareMap.get(DcMotorEx.class, "rightBack");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        //Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeft.setVelocityPIDFCoefficients(15, 0, 0, 0);
        frontRight.setVelocityPIDFCoefficients(15, 0, 0, 0);
        backLeft.setVelocityPIDFCoefficients(15, 0, 0, 0);
        backRight.setVelocityPIDFCoefficients(15, 0, 0, 0);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);
    }


    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("currentAngle: ", getAngle());

        move(gamepad1.left_stick_x, gamepad1.left_stick_y);


        leftFrontPower = move + turn;
        leftBackPower = move + turn;
        rightFrontPower = move - turn;
        rightBackPower = move - turn;

        frontLeft.setVelocity(leftFrontPower * 6000);
        frontRight.setVelocity(rightFrontPower * 6000);
        backLeft.setVelocity(leftBackPower * 6000);
        backRight.setVelocity(rightBackPower * 6000);


        if (gamepad1.dpad_down && gamepad1.a) {
            OffSet();
        }
    }


    public double getAngle() {
        double angle = (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) + offSetAngle;

        if (angle < 0){
            angle += 360;
        }

        return angle;
    }

    public void move(double x, double y) {
        //Target angle finding
        if (x != 0 || y != 0) {
            angle = Math.atan2(y, -x);  // Calculate the angle in radians
        }
        target = (int) Math.toDegrees(angle);  // Convert the angle to degrees
        if (target < 0) {
            target += 360;  // Adjust the angle to be within the 0-360 degree range
        }
        target += 90;  // Shift the angle by 270 degrees
        if (target >= 360) {
            target -= 360;  // Wrap the angle within the 0-360 degree range
        }

        if (target == 0){
            target = 355;
        }



        telemetry.addData("target angle",target);
        //turning
        int angleDiff = (int) (target - getAngle());
//        if (angleDiff > 180){
//            angleDiff += 360;
//        } else if (angleDiff < -180){
//            angleDiff -= 360;
//        }
        telemetry.addData("angle distance",angleDiff);
        if (Math.abs(angleDiff) > 15){
            if (angleDiff > 0){
                turn = 0.5;
            }else{
                turn = -0.5;
            }
        }else{
            if (angleDiff > 0){
                turn = 0.05;
            }else if(angleDiff < 0){
                turn = -0.05;
            }else{
                turn = 0;
            }
        }


        //movement
        move = -(Math.sqrt(Math.pow(x,2) + Math.pow(y,2))) * 0.4;


    }


    public void OffSet() {
        offSetAngle = 90 - getAngle();
    }

}