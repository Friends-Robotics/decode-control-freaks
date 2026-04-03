package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //in KG weigh the robot to get this value exact!!!
            .mass(11.2)
            // update forward and lateral values.
            .forwardZeroPowerAcceleration(-47.45532227516153) //take the value from panels
            .lateralZeroPowerAcceleration(33.13096486489604) //take tuned value from panels
            //PIDF all need tuned!!!
            .translationalPIDFCoefficients(new PIDFCoefficients(0.11, 0,0.009, 0.018 )) //go to manual on driver hub and select translational tuner
            //try to find a for F that the motors whine but don't move, for P does it over or under correct? D, speed it returns to its position at
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.2, 0.018)) // go to manual and into heading tuner test.
            //start with P value, then do the D value,
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0.0, 0.00001,0.6,0.01    ))// go to tests, then line
            //find path constraints, start with breaking strength
            //go to drivePIDF set everytthing to zero apart from T and F, Start with P then edit D with it(might have to go back and edit the headingPIDF)
            .centripetalScaling(0.0005); //manual ->centripetaltuner
    //should find this value at the top (only need to modify the scaling)

    //after tuning everything run the line test again to se how well the robot holds the line, may have to go throug hand tune values again.


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRM")
            .rightRearMotorName("BRM")
            .leftRearMotorName("BLM")
            .leftFrontMotorName("FLM")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            //Check that the motor directions are correct
            //This x and y velocity needs changed!!!
            .xVelocity(39.85210316575418) //forward velocity tuner value
            .yVelocity(32.64262978861652); //lateral velocity tuner value

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //in inches!!!!
            .forwardPodY(-6.5)
            .strafePodX(4.0)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint") //check/update name
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            //check if forward or backwards if backwards change "FORWARD to REVERSED"
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED); //check if forward or backwards

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            0.7,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
