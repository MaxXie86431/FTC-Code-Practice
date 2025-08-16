package org.firstinspires.ftc.teamcode.robot.components;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinearSlide {
    private DcMotorEx linearSlide;
    private static final int lowerLimit = 0;
    private static final int upperLimit = 1000;

    public LinearSlide(HardwareMap hardwareMap) {
        linearSlide = hardwareMap.get(DcMotorEx.class, "Linear Slide");
    }

    public void moveLinearSlide(double power) {
        if (power<0 && linearSlide.getCurrentPosition() <= lowerLimit) {
            power = 0;
        } else if (power>0 && linearSlide.getCurrentPosition() >= upperLimit) {
            power = 0;
        }
        linearSlide.setPower(power);
    }

    public void moveLinearSlideToPosition(int position) { //position in ticks
        if (position <= lowerLimit) {
            position = lowerLimit;
        } else if (position>=upperLimit) {
            position = upperLimit;
        }

        linearSlide.setTargetPosition(position);
        linearSlide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
    }

    public Action moveLinearSlideToPositionAction(int position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                moveLinearSlideToPosition(position);
                return Math.abs(linearSlide.getCurrentPosition() - position) > 50;
            }
        };
    }
}
