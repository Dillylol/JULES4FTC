package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
        public static FollowerConstants followerConstants = new FollowerConstants()
                        .mass(3.17514659);
        public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

        public static MecanumConstants driveConstants = new MecanumConstants()
                        .maxPower(1)
                        .rightFrontMotorName("rf")
                        .rightRearMotorName("rr")
                        .leftRearMotorName("lr")
                        .leftFrontMotorName("lf")
                        .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

        public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
                        .rightFrontMotorName("rf")
                        .rightRearMotorName("rr")
                        .leftRearMotorName("lr")
                        .leftFrontMotorName("lf")
                        .leftFrontEncoderDirection(Encoder.FORWARD)
                        .leftRearEncoderDirection(Encoder.FORWARD)
                        .rightFrontEncoderDirection(Encoder.REVERSE)
                        .rightRearEncoderDirection(Encoder.REVERSE)
                        .robotWidth(17)
                        .robotLength(16);

        public static Follower createFollower(HardwareMap hardwareMap) {
                return new FollowerBuilder(followerConstants, hardwareMap)
                                .pathConstraints(pathConstraints)
                                .mecanumDrivetrain(driveConstants)
                                .driveEncoderLocalizer(localizerConstants)
                                .build();
        }
}