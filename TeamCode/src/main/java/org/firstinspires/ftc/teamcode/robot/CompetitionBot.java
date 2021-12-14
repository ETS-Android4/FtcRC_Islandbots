package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class CompetitionBot {

    // Motors
    public DcMotor RFmotor, RBmotor, LFmotor, LBmotor;
    public DcMotor DuckWheel, Intake, LinearSlide;

    // Servos
    public Servo BoxServo;

    // Constants
    public static final double MAX_SPEED = 0.7;

    public static final double DUCK_POWER = 0.5;

    public static final double INTAKE_POWER = 0.5;

    public static final double BOX_IN = 0.05;
    public static final double BOX_OUT = 0.5;

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

        // Drive Train Directions
        RFmotor.setDirection(DcMotor.Direction.REVERSE);
        RBmotor.setDirection(DcMotor.Direction.REVERSE);

        // Servo Initial Positions


        telemetry.addData("Status", "Successfully Initialized");
        telemetry.update();
    }

    public double[] tankMove(double joystickMove, double joystickTurn, Telemetry telemetry) {
        double SPEED_REDUCTION = MAX_SPEED;

        double L = joystickMove * joystickMove + (0.5*joystickTurn);
        double R = joystickMove * joystickMove - (0.5*joystickTurn);

        L *= SPEED_REDUCTION;
        R *= SPEED_REDUCTION;

        double[] powerVals = {L, R};

        L *= clamp(powerVals);
        R *= clamp(powerVals);

        setMotors(L, R);

        return powerVals;
    }

    private double clamp(double[] powerVals) {
        double max = Math.max(Math.max(powerVals[0], powerVals[1]), Math.max(powerVals[2], powerVals[3]));
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
