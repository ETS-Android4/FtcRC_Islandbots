package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class CompetitionBot {

    // Sensors
    public BNO055IMU gyro;

    // Motors
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor;
    public DcMotor DuckWheel, Intake, LinearSlide;

    // Servos
    public Servo BoxServo;

    // Constants
    public static final double MAX_SPEED = 0.7;
    public static final double FAST_SPEED = 1;
    public static final double SLOW_SPEED = 0.2;
    public static final double TURN_SPEED = 0.7;

    public static final double IN_TO_POS = 12;

    public static final double DUCK_POWER = .75;

    public static final double INTAKE_POWER = 1;

    public static final double BOX_VERT = 0.74;
    public static final double BOX_SLANT = 0.64;
    public static final double BOX_HORIZ = 0.44;
    public static final double BOX_OUT = 0.3;
    public static final double BOX_OUT2 = 0.16;

    public static final double[] SLIDE_STAGES = {};

    public CompetitionBot(HardwareMap hwMap, Telemetry telemetry) {

        // Hardware Declarations
        RFmotor = hwMap.dcMotor.get("RFmotor");
        RBmotor = hwMap.dcMotor.get("RBmotor");
        LFmotor = hwMap.dcMotor.get("LFmotor");
        LBmotor = hwMap.dcMotor.get("LBmotor");

        DuckWheel = hwMap.dcMotor.get("DuckWheel");
        Intake = hwMap.dcMotor.get("Intake");
        LinearSlide = hwMap.dcMotor.get("LinearSlide");

        BoxServo = hwMap.servo.get("Box");

        gyro = hwMap.get(BNO055IMU.class, "gyro");

        // Motor Setup
        LFmotor.setDirection(DcMotor.Direction.REVERSE);
        LBmotor.setDirection(DcMotor.Direction.REVERSE);

        DuckWheel.setDirection(DcMotor.Direction.REVERSE);
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        LinearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Servo Initial Positions
        BoxServo.setPosition(BOX_VERT);

        // Gyro Initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyro.initialize(parameters);

        telemetry.addData("Status", "Successfully Initialized");
        telemetry.update();
    }

    public void tankMove(double joystickMove, double joystickTurn, boolean fastMode, boolean slowMode, Telemetry telemetry) {
        double SPEED_REDUCTION = MAX_SPEED;
        if (fastMode) SPEED_REDUCTION = FAST_SPEED;
        else if (slowMode) SPEED_REDUCTION = SLOW_SPEED;

        double L = joystickMove * Math.abs(joystickMove) - (TURN_SPEED * joystickTurn);
        double R = joystickMove * Math.abs(joystickMove) + (TURN_SPEED * joystickTurn);

        double[] powerVals = {L, R};
        double coef = clamp(powerVals) * SPEED_REDUCTION;

        L *= coef;
        R *= coef;

        setMotors(L, R);
    }

    public double getPitch() { return gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; }

    private double clamp(double[] powerVals) {
        double max = Math.abs(Math.max(powerVals[0], powerVals[1]));
        if (max > 1) {
            return 1 / max;
        } else {
            return 1;
        }
    }

    public void setMotors(double L, double R) {
        LFmotor.setPower(L);
        LBmotor.setPower(L);
        RFmotor.setPower(R);
        RBmotor.setPower(R);
    }
}
