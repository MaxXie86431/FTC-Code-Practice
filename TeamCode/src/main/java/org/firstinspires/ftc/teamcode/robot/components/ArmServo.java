package org.firstinspires.ftc.teamcode.robot.components;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmServo {
    private final Servo armServo;
    private boolean moved = false;
    private boolean lastState = false;

    public ArmServo(HardwareMap hardwareMap) {
        armServo = hardwareMap.get(Servo.class, "Arm Servo");
        armServo.setPosition(0);
    }

    public void moveArmUp() {
        armServo.setPosition(1);
    }

    public void moveArmDown() {
        armServo.setPosition(0);
    }

    public Action moveArmUpAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveArmUp();
                return false;
            }
        };
    }

    public Action moveArmDownAction() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveArmDown();
                return false;
            }
        };
    }
}

