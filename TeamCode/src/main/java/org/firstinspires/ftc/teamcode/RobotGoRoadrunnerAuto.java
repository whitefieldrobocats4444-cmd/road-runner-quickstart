package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

@Autonomous(name="RobotGoRoadrunnerAuto", group="Autonomous")
public class RobotGoRoadrunnerAuto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor RearRightDrive = null;
    private DcMotor RearLeftDrive = null;
    private IMU GyroscopeDrive = null;
    private HuskyLens huskyLens;
    private DcMotor perp = null;
    private DcMotor OuttakeDrive1 = null;
    private DcMotor OuttakeDrive2 = null;
    private CRServo FeederDrive1 = null;
    public double rightPowerY;
    public double leftPowerY;
    public double rightPowerX;
    public double leftPowerX;
    public double yaw;
    public double alignX;
    public double alignY;
    public double speed;
    public double oldSpeed;
    public double encoderReset;
    public double currentPosition;
    public double kP;
    public double kD;
    public double kI;
    public boolean beginCountdown;

    public void GoDirection (double x, double y, double speed) {
        for (int i = 0; i < 5; i++) {

        }
    }

    public void align (double x, double y) {

    }

    public void orient (double alignYaw) {
        while (Math.abs(yaw - alignYaw) > 1) {
            GyroscopeDrive.getRobotYawPitchRollAngles();
            YawPitchRollAngles orientation = GyroscopeDrive.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw(AngleUnit.DEGREES);
            FrontLeftDrive.setPower((yaw - alignYaw) / -60);
            RearLeftDrive.setPower((yaw - alignYaw) / -60);
            FrontRightDrive.setPower((yaw - alignYaw) / 60);
            RearRightDrive.setPower((yaw - alignYaw) / 60);
        }
    }

    public void shoot (double power, double countdown) {
        OuttakeDrive2.getCurrentPosition();
        encoderReset = OuttakeDrive2.getCurrentPosition();
        while (countdown > 0) {
            OuttakeDrive2.getCurrentPosition();
            currentPosition = OuttakeDrive2.getCurrentPosition()-encoderReset;
            speed = currentPosition-oldSpeed;
            kP = (power-speed)/5000;
            kI = kI + (kP) * 2;
            OuttakeDrive1.setPower(kP+kD+kI);
            OuttakeDrive2.setPower(kP+kD+kI);
            kD = kP;
            oldSpeed = currentPosition;
            telemetry.addData("OuttakePower: ", OuttakeDrive2.getPower());
            telemetry.addData("speed: ", speed);
            telemetry.update();
            if (Math.abs(power/speed) > 1.1 || Math.abs(power/speed) < 0.9) {
                beginCountdown = true;
            }
            if (beginCountdown) {
                countdown = countdown-1;
            }
        }
    }

    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        telemetry.addData("Status", "Initialized");
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        RearLeftDrive = hardwareMap.get(DcMotor.class, "RearLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        RearRightDrive = hardwareMap.get(DcMotor.class, "RearRightDrive");
        GyroscopeDrive = hardwareMap.get(IMU.class, "GyroscopeDrive");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        perp = hardwareMap.get(DcMotor.class, "perp");
        OuttakeDrive1 = hardwareMap.get(DcMotor.class, "OuttakeDrive1");
        OuttakeDrive2 = hardwareMap.get(DcMotor.class, "OuttakeDrive2");
        FeederDrive1 = hardwareMap.get(CRServo.class, "FeederDrive1");
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        RearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perp.setDirection(DcMotor.Direction.FORWARD);
        OuttakeDrive1.setDirection(DcMotor.Direction.REVERSE);
        OuttakeDrive2.setDirection(DcMotor.Direction.FORWARD);
        FeederDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        GyroscopeDrive.initialize(new IMU.Parameters(orientationOnRobot));
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        telemetry.addData("Status", "Initialized");
        GyroscopeDrive.resetYaw();

        waitForStart();
        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen
                )
        );
    }
}
