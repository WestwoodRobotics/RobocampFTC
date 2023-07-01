package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MecanumRobot extends OpMode {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void init() {
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


    }

    @Override
    public void loop() {
        double leftFrontPower;
        double rightFrontPower;
        double leftBackPower;
        double rightBackPower;

        leftFrontPower = 1.5 * (-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
        rightFrontPower = 1.5 * (-gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x);
        leftBackPower = 1.5 * (-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
        rightBackPower = 1.5 * (-gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

        frontLeft.setVelocity(leftFrontPower * 3000);
        frontRight.setVelocity(rightBackPower * 3000);
        backLeft.setVelocity(leftBackPower * 3000);
        backRight.setVelocity(rightFrontPower * 3000);
    }
}
