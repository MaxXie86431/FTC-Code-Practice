package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.components.*;
import org.firstinspires.ftc.teamcode.autonomous.AutoConfig;
import org.firstinspires.ftc.teamcode.autonomous.AprilTagLocalizer;

@Autonomous(name = "First Autonomous")
public class FirstAutonomous extends LinearOpMode {
    private MecanumDrive drive;
    private ArmServo armServo;
    private LinearSlide linearSlide;
    private AprilTagLocalizer aprilTagLocalizer;
    Pose2d startPose = new Pose2d(0, 0, 0);


    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, startPose);
        armServo = new ArmServo(hardwareMap);
        linearSlide = new LinearSlide(hardwareMap);
        drive.localizer.setPose(startPose);
        AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        bucket1()

                )
        );
    }

    private Action bucket1() {
        return new SequentialAction(
            new ParallelAction (
                drive.actionBuilder(drive.localizer.getPose())
                        .splineTo(AutoConfig.strafe1, Math.toRadians(90))
                        .build(),
                linearSlide.moveLinearSlideToPositionAction(1000)
            ),

            new ParallelAction (
                armServo.moveArmUpAction(),
                aprilTagLocalizer.aprilTagPose(this, 500, telemetry, drive)
            )
        );
    }

}