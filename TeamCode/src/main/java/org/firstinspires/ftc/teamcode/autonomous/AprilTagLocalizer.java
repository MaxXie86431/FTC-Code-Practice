package org.firstinspires.ftc.teamcode.autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.BooleanSupplier;

public class AprilTagLocalizer {
    private static final boolean USE_WEBCAM = true;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTagLocalizer(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    public Action aprilTagPose(LinearOpMode opMode, double duration, Telemetry telemetry, MecanumDrive drive) {
        return new Action() {
            List<Pose2d> detectedPoses = new ArrayList<>();
            ElapsedTime timer = new ElapsedTime();
            boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }

                if (opMode.opModeIsActive() && timer.milliseconds() < duration) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    telemetry.addData("# AprilTags Detected", currentDetections.size());

                    // Step through the list of detections and display info for each one.
                    for (AprilTagDetection detection : currentDetections) {
                        if (detection.metadata != null) {
                            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                                    detection.robotPose.getPosition().x,
                                    detection.robotPose.getPosition().y,
                                    detection.robotPose.getPosition().z));
                            telemetry.addLine(String.format("PRY %6.1f %6s.1f %6.1f  (deg)",
                                    detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                                    detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                                    detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));

                            if (detection.id == 13) {
                                detectedPoses.add(new Pose2d(detection.robotPose.getPosition().x, detection.robotPose.getPosition().y, detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                            }
                        } else {
                            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                        }


                    }   // end for() loop

                    // Add "key" information to telemetry
                    telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
                    telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

                    opMode.idle();
                    return true;
                } else {
                    if (detectedPoses.isEmpty()) {
                        return false;
                    } else {
                        Pose2d aprilTagPose = separatePose(detectedPoses);
                        drive.localizer.setPose(aprilTagPose);
                        return false;
                    }
                }
            }
        };
    }

    public static Pose2d separatePose(List<Pose2d> poses) {
        List<Double> xList = new ArrayList<>();
        List<Double> yList = new ArrayList<>();
        List<Double> yawList = new ArrayList<>();

        for (Pose2d pose : poses) {
            xList.add(pose.position.x);
            yList.add(pose.position.y);
            yawList.add(pose.heading.toDouble());
        }
        return new Pose2d(median(xList), median(yList), median(yawList));
    }

    public static double median(List<Double> list) {
        Collections.sort(list);
        int n = list.size();
        if (n % 2 == 1){
            return list.get(n/2);
        } else {
            return (list.get(n/2-1) + list.get(n/2))/2;
        }
    }
}
