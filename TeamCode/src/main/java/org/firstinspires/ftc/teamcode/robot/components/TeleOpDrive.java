package org.firstinspires.ftc.teamcode.robot.components;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TeleOpDrive {

    private final DcMotor leftFront, rightFront, leftBack, rightBack;

    public TeleOpDrive(HardwareMap hardwareMap) {


        //Initialize motors
        leftFront = hardwareMap.get(DcMotorEx.class, "Top-Left-Motor");
        rightFront = hardwareMap.get(DcMotorEx.class, "Top-Right-Motor");
        leftBack = hardwareMap.get(DcMotorEx.class, "Bottom-Left-Motor");
        rightBack = hardwareMap.get(DcMotorEx.class, "Bottom-Right-Motor");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    public void drive(double forward, double strafe, double rotate) {
        forward = -forward;
        strafe = strafe * 1.1;

        double frontLeftPower = forward + strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backLeftPower = forward - strafe + rotate;
        double backRightPower = forward + strafe - rotate;

        double max = Math.max(
            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        );

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        leftFront.setPower(frontLeftPower);
        rightFront.setPower(frontRightPower);
        leftBack.setPower(backLeftPower);
        rightBack.setPower(backRightPower);
    }


}