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

@TeleOp(name = "MecanumField")
public class MecanumField extends OpMode {
    double rTheta = 0;
    double rThetaRad = 0;
    double nX = 0;
    double nY = 0;
    double orgAngle;
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
    double offSetAngle = 0;
    double currentActualAngle = 0;

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

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double straight =
                gamepad1.left_stick_y;
        double strafing = -gamepad1.left_stick_x;
        double turn = -(gamepad1.right_stick_x) * 0.7;

        this.calcNewXY(strafing, straight);
        if (gamepad1.left_bumper) {
            leftFrontPower = 1.5 * (-nY - nX - turn);
            rightFrontPower = 1.5 * (-nY - nX + turn);
            leftBackPower = 1.5 * (-nY + nX - turn);
            rightBackPower = 1.5 * (-nY + nX + turn);

        } else if (gamepad1.right_bumper) {
            leftFrontPower = 0.2 * (-nY - nX - turn);
            rightFrontPower = 0.2 * (-nY - nX + turn);
            leftBackPower = 0.2 * (-nY + nX - turn);
            rightBackPower = 0.2 * (-nY + nX + turn);

        } else {
            leftFrontPower = 0.4 * (-nY - nX - turn);
            rightFrontPower = 0.4 * (-nY - nX + turn);
            leftBackPower = 0.4 * (-nY + nX - turn);
            rightBackPower = 0.4 * (-nY + nX + turn);
        }

        frontLeft.setVelocity(leftFrontPower * 3000);
        frontRight.setVelocity(rightBackPower * 3000);
        backLeft.setVelocity(leftBackPower * 3000);
        backRight.setVelocity(rightFrontPower * 3000);

        telemetry.addData("newX: ", strafing);
        telemetry.addData("newY: ", straight);
        telemetry.addData("turn: ", turn);
        telemetry.addData("currentAngle: ", currentActualAngle);

        if (gamepad1.dpad_down && gamepad1.a) {
            OffSet();
        }
    }

    public void calcNewXY(double x, double y) {
        orgAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        orgAngle = orgAngle + 90;
        if (orgAngle < 0) {
            orgAngle = orgAngle + 360;
        }

        if (orgAngle > 360) {
            orgAngle = orgAngle - 360;
        }
        rTheta = orgAngle + offSetAngle;
        rThetaRad = rTheta * (Math.PI / 180.0);
        double cosTheta = Math.cos(rThetaRad);
        double sinTheta = Math.sin(rThetaRad);
        nX = (x * sinTheta) - (y * cosTheta);
        nY = (x * cosTheta) + (y * sinTheta);
    }


    @Override
    public void stop() {
    }

    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    }


    public void OffSet() {
        offSetAngle = 90 - orgAngle;
    }

}