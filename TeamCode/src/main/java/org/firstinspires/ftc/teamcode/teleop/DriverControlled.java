package org.firstinspires.ftc.teamcode.teleop;
import org.firstinspires.ftc.teamcode.robot.components.ArmServo;
import org.firstinspires.ftc.teamcode.robot.components.TeleOpDrive;
import org.firstinspires.ftc.teamcode.robot.components.TeleOpDrive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name = "Driver Controlled")
public class DriverControlled extends OpMode {
    private ArmServo armServo;
    private TeleOpDrive driveTrain;

    //Arm Servo initial booleans
    private boolean armState = false; //true = up, false = down
    private boolean lastButtonState = false;

    @Override
    public void init() {
        armServo = new ArmServo(hardwareMap);
        driveTrain = new TeleOpDrive(hardwareMap);

    }

    @Override
    public void loop() {
        // GAMEPAD1 A - ARM SERVO MOVEMENT
        if (!armState && !lastButtonState && gamepad1.a) {
            armServo.moveArmUp();
            armState = true;
        } else if (armState && !lastButtonState && gamepad1.a) {
            armServo.moveArmDown();
            armState = false;
        }
        lastButtonState = gamepad1.a;

        //DRIVETRAIN
        driveTrain.drive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x);
    }
}