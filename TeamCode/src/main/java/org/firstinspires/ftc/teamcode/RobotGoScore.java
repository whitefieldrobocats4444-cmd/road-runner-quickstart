package org.firstinspires.ftc.teamcode;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import android.transition.Slide;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="RobotGoScore", group="TeleOp")
public class RobotGoScore extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontRightDrive = null;
    private DcMotor FrontLeftDrive = null;
    private DcMotor RearRightDrive = null;
    private DcMotor RearLeftDrive = null;
    private DcMotor par = null;
    private DcMotor perp = null;
    private DcMotor OuttakeDrive1 = null;
    private DcMotor OuttakeDrive2 = null;
    private IMU GyroscopeDrive = null;
    private HuskyLens huskyLens;
    private CRServo FeederDrive1 = null;

    public void init() {
        telemetry.addData("Status", "Initialized");
        FrontLeftDrive = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        RearLeftDrive = hardwareMap.get(DcMotor.class, "RearLeftDrive");
        FrontRightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");
        RearRightDrive = hardwareMap.get(DcMotor.class, "RearRightDrive");
        perp = hardwareMap.get(DcMotor.class, "perp");
        OuttakeDrive1 = hardwareMap.get(DcMotor.class, "OuttakeDrive1");
        OuttakeDrive2 = hardwareMap.get(DcMotor.class, "OuttakeDrive2");
        GyroscopeDrive = hardwareMap.get(IMU.class, "GyroscopeDrive");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        FeederDrive1 = hardwareMap.get(CRServo.class, "FeederDrive1");
        FrontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        RearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        FrontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        RearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        perp.setDirection(DcMotor.Direction.FORWARD);
        OuttakeDrive1.setDirection(DcMotor.Direction.REVERSE);
        OuttakeDrive2.setDirection(DcMotor.Direction.FORWARD);
        FeederDrive1.setDirection(CRServo.Direction.REVERSE);
        //FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        OuttakeDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        GyroscopeDrive.initialize(new IMU.Parameters(orientationOnRobot));
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "Initialized");

        GyroscopeDrive.resetYaw();
    }

    public void start() {
        runtime.reset();
    }
    double rightPowerY;
    double leftPowerY;
    double rightPowerX;
    double leftPowerX;
    double yaw;
    double sine;
    double cosine;
    double alignX;
    double alignY;
    double kP;
    double kD;
    double kI;
    double power;
    double speed;
    double oldSpeed;
    double currentPosition;
    boolean Simon;

    public void loop() {

        perp.setPower(gamepad1.left_trigger);
        OuttakeDrive1.setPower(gamepad1.right_trigger * 0.8);
        OuttakeDrive2.setPower(gamepad1.right_trigger * 0.8);

        if (gamepad1.x) {
            FeederDrive1.setPower(-1);
        } else {
            FeederDrive1.setPower(0);
        }

        rightPowerY = gamepad1.right_stick_y;
        leftPowerY = gamepad1.left_stick_y;
        rightPowerX = gamepad1.right_stick_x;
        leftPowerX = gamepad1.left_stick_x;

        YawPitchRollAngles orientation = GyroscopeDrive.getRobotYawPitchRollAngles();

        sine = Math.sin(orientation.getYaw(AngleUnit.RADIANS));
        cosine = Math.cos(orientation.getYaw(AngleUnit.RADIANS));

        if (gamepad1.dpad_left) {
            yaw = yaw + 1;
        }

        if (yaw > 180) {
            yaw = -180;
        }

        HuskyLens.Block[] blocks = huskyLens.blocks();
        telemetry.addData("Block count", blocks.length);
        for (int i = 0; i < blocks.length; i++) {
            telemetry.addData("Block", blocks[i].toString());
            telemetry.addData("Block Center", blocks[i].x);
            telemetry.addData("Block Center", blocks[i].y);
            telemetry.addData("Block Edge Left", blocks[i].left);
            telemetry.addData("Block Edge Top", blocks[i].top);
        }

        if (gamepad1.a) {
            for (int i = 0; i < blocks.length; i++) {
                if (blocks[i].id == 1) {
                    telemetry.addData(">>", "Pattern = Green Purple Purple");
                }
                if (blocks[i].id == 2) {
                    telemetry.addData(">>", "Pattern = Purple Green Purple");
                }
                if (blocks[i].id == 3) {
                    telemetry.addData(">>", "Pattern = Purple Purple Green");
                }
                if (blocks[i].id == 4) {
                    alignX = blocks[i].x - 150;
                    alignY = blocks[i].y - 63;
                    if (orientation.getYaw() < -40.338 || orientation.getYaw() > -38.338 || alignX < -0.5 || alignX > 0.5 || alignY < -0.5 || alignY > 0.5) {
                        telemetry.addData(">>", "Locking on blue team goal");
                        Simon = true;
                        yaw = -39.338;
                        FrontLeftDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90) - (alignX/60) - (alignY/60));
                        RearLeftDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90) - (alignX/-60) - (alignY/60));
                        FrontRightDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90) - (alignX/-60) - (alignY/60));
                        RearRightDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90) - (alignX/60) - (alignY/60));
                    }
                }
                if (blocks[i].id == 5) {
                    alignX = blocks[i].x - 190;
                    alignY = blocks[i].y - 50;
                    if (orientation.getYaw() < -51.662 || orientation.getYaw() > -49.662 || alignX < -1 || alignX > 1 || alignY < -0.5 || alignY > 0.5) {
                        telemetry.addData(">>", "Locking on red team goal");
                        Simon = true;
                        yaw = -50.662;
                        FrontLeftDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90) - (alignX/80) - (alignY/60));
                        RearLeftDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90) - (alignX/-80) - (alignY/60));
                        FrontRightDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90) - (alignX/-80) - (alignY/60));
                        RearRightDrive.setPower(((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90) - (alignX/80) - (alignY/60));
                    }
                }
            }
        }

        /*if (gamepad1.dpad_right & !Simon) {
            FrontLeftDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90);
            RearLeftDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90);
            FrontRightDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90);
            RearRightDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90);
        } else if (!Simon & gamepad1.left_bumper) {
            FrontLeftDrive.setPower((leftPowerY + (-leftPowerX))/5);
            RearLeftDrive.setPower((leftPowerY + leftPowerX)/5);
            FrontRightDrive.setPower((rightPowerY + rightPowerX)/5);
            RearRightDrive.setPower((rightPowerY + (-rightPowerX))/5);
        } else if (!Simon) {
            FrontLeftDrive.setPower(leftPowerY + (-leftPowerX));
            RearLeftDrive.setPower(leftPowerY + leftPowerX);
            FrontRightDrive.setPower(rightPowerY + rightPowerX);
            RearRightDrive.setPower(rightPowerY + (-rightPowerX));
        }*/

        if (gamepad1.dpad_right & !Simon) {
            FrontLeftDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90);
            RearLeftDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / -90);
            FrontRightDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90);
            RearRightDrive.setPower((orientation.getYaw(AngleUnit.DEGREES) - yaw) / 90);
        } else if (leftPowerY == 0 & leftPowerX == 0 & !Simon) {
            FrontLeftDrive.setPower(-rightPowerX);
            RearLeftDrive.setPower(-rightPowerX);
            FrontRightDrive.setPower(rightPowerX);
            RearRightDrive.setPower(rightPowerX);
        } else if (gamepad1.left_bumper & !Simon) {
            FrontLeftDrive.setPower((-rightPowerX + (((leftPowerY*Math.abs(leftPowerY)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)) + (-leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) - sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)))/5);
            RearLeftDrive.setPower((-rightPowerX + (((leftPowerY*Math.abs(leftPowerY) * (cosine * Math.abs(cosine) - sine * Math.abs(sine))) + (leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)))/5);
            FrontRightDrive.setPower((rightPowerX + (((leftPowerY*Math.abs(leftPowerY)) * (cosine * Math.abs(cosine) - sine * Math.abs(sine)) + (leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)))/5);
            RearRightDrive.setPower((rightPowerX + (((leftPowerY*Math.abs(leftPowerY)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)) + (-leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) - sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)))/5);
        } else if (!Simon) {
            FrontLeftDrive.setPower(-rightPowerX + (((leftPowerY*Math.abs(leftPowerY)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)) + (-leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) - sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)));
            RearLeftDrive.setPower(-rightPowerX + (((leftPowerY*Math.abs(leftPowerY) * (cosine * Math.abs(cosine) - sine * Math.abs(sine))) + (leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)));
            FrontRightDrive.setPower(rightPowerX + (((leftPowerY*Math.abs(leftPowerY)) * (cosine * Math.abs(cosine) - sine * Math.abs(sine)) + (leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)));
            RearRightDrive.setPower(rightPowerX + (((leftPowerY*Math.abs(leftPowerY)) * (cosine * Math.abs(cosine) + sine * Math.abs(sine)) + (-leftPowerX*Math.abs(leftPowerX)) * (cosine * Math.abs(cosine) - sine * Math.abs(sine)))/Math.sqrt(leftPowerY*leftPowerY + leftPowerX*leftPowerX)));
        }

        Simon = false;

        telemetry.addData("Rear Right Motor: ", RearRightDrive.getCurrentPosition());
        telemetry.addData("Front Right Motor: ", FrontRightDrive.getCurrentPosition());
        telemetry.addData("Rear Left Motor: ", RearLeftDrive.getCurrentPosition());
        telemetry.addData("Front Left Motor: ", FrontLeftDrive.getCurrentPosition());
        telemetry.addData("Yaw: ", orientation.getYaw(AngleUnit.DEGREES));
        telemetry.addData("Expected Yaw Value", yaw);
        telemetry.addData("Sine: ", sine);
        telemetry.addData("Cosine: ", cosine);
        telemetry.addData("Rear Right Motor Power: ", RearRightDrive.getPower());
        telemetry.addData("Front Right Motor Power: ", FrontRightDrive.getPower());
        telemetry.addData("Rear Left Motor Power: ", RearLeftDrive.getPower());
        telemetry.addData("Front Left Motor Power: ", FrontLeftDrive.getPower());
        telemetry.addData("Outtake Drive 1: ", OuttakeDrive1.getCurrentPosition());
        telemetry.addData("Outtake Drive 2: ", OuttakeDrive2.getCurrentPosition());
        telemetry.addData("Speed: ", speed);
    }
}