package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Mecanum drivetrain: robot-centric and field-centric driving, plus simple
 * encoder moves and IMU turns for autonomous.
 *
 * Hardware configuration names (set these on the Driver Hub > Configure Robot):
 *   Motors: "frontLeft", "frontRight", "backLeft", "backRight"
 *   IMU:    "imu"  (the Control Hub's built-in IMU)
 *
 * Wheel layout seen from ABOVE must form an "X" with the rollers:
 *      frontLeft  \\   //  frontRight
 *      backLeft   //   \\  backRight
 */
public class MecanumDrive {

    // ---------------- Tune these for YOUR robot ----------------

    /** Which way the REV logo and USB ports on the Control Hub face. */
    public static RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    /** Encoder ticks per output-shaft revolution. goBILDA 5203 312 RPM = 537.7, REV HD Hex 20:1 = 560. */
    public static double TICKS_PER_REV = 537.7;
    /** Wheel diameter in inches. goBILDA 104 mm mecanum = 4.094 in, 96 mm = 3.78 in. */
    public static double WHEEL_DIAMETER_IN = 4.094;
    /** External gear ratio between motor and wheel (1.0 = direct drive). */
    public static double GEAR_RATIO = 1.0;
    /** Mecanum wheels slip sideways; scale strafe to make it match forward distance. Tune 1.0-1.2. */
    public static double STRAFE_MULTIPLIER = 1.1;
    /** Proportional gain for IMU turns (power per degree of error). */
    public static double TURN_KP = 0.02;

    public static double ticksPerInch() {
        return (TICKS_PER_REV * GEAR_RATIO) / (WHEEL_DIAMETER_IN * Math.PI);
    }

    // -----------------------------------------------------------

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final DcMotorEx[] motors;
    private final IMU imu;
    private double maxPower = 1.0;

    public MecanumDrive(HardwareMap hardwareMap) {
        frontLeft  = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight  = hardwareMap.get(DcMotorEx.class, "backRight");
        motors = new DcMotorEx[]{frontLeft, frontRight, backLeft, backRight};

        // Most drivetrains need the left side reversed. If the robot spins when you
        // push forward, or a wheel runs backwards, fix it HERE (use MotorDirectionTest).
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));
        imu.resetYaw();
    }

    // ---------------- TeleOp driving ----------------

    /**
     * Robot-centric drive. Inputs are -1..1.
     * @param forward + drives forward
     * @param strafe  + strafes right
     * @param turn    + turns clockwise
     */
    public void drive(double forward, double strafe, double turn) {
        setWheelPowers(forward, strafe * STRAFE_MULTIPLIER, turn);
    }

    /** Field-centric drive: "forward" on the stick is always away from the driver. */
    public void driveFieldCentric(double forward, double strafe, double turn) {
        double heading = getHeadingRadians();
        // Rotate the joystick vector by -heading
        double rotStrafe  = strafe * Math.cos(-heading) - forward * Math.sin(-heading);
        double rotForward = strafe * Math.sin(-heading) + forward * Math.cos(-heading);
        setWheelPowers(rotForward, rotStrafe * STRAFE_MULTIPLIER, turn);
    }

    private void setWheelPowers(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        // Scale down so no wheel exceeds 1.0 while keeping the direction of motion
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeft.setPower(maxPower * fl / max);
        frontRight.setPower(maxPower * fr / max);
        backLeft.setPower(maxPower * bl / max);
        backRight.setPower(maxPower * br / max);
    }

    /** 0..1 cap on drive power (e.g. 0.4 for slow/precision mode). */
    public void setMaxPower(double power) {
        maxPower = Range.clip(power, 0.0, 1.0);
    }

    public void stop() {
        for (DcMotorEx m : motors) m.setPower(0);
    }

    // ---------------- IMU ----------------

    /** Heading in radians, CCW positive, 0 = direction robot faced at last reset. */
    public double getHeadingRadians() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    // ---------------- Autonomous helpers ----------------

    /**
     * Drive a relative distance using encoders (blocking). Forward and strafe can be combined
     * for diagonal moves. Returns early if the OpMode is stopped or the timeout passes.
     */
    public void driveDistance(LinearOpMode opMode, double forwardIn, double strafeIn,
                              double power, double timeoutSec) {
        double strafeAdj = strafeIn * STRAFE_MULTIPLIER;
        double tpi = ticksPerInch();
        int[] deltas = {
                (int) Math.round((forwardIn + strafeAdj) * tpi), // FL
                (int) Math.round((forwardIn - strafeAdj) * tpi), // FR
                (int) Math.round((forwardIn - strafeAdj) * tpi), // BL
                (int) Math.round((forwardIn + strafeAdj) * tpi)  // BR
        };

        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + deltas[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        double p = Math.abs(power);
        for (DcMotorEx m : motors) m.setPower(p);

        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec && anyBusy()) {
            opMode.telemetry.addData("Driving", "fwd %.1f in, strafe %.1f in", forwardIn, strafeIn);
            opMode.telemetry.addData("FL pos/target", "%d / %d",
                    frontLeft.getCurrentPosition(), frontLeft.getTargetPosition());
            opMode.telemetry.update();
        }

        stop();
        for (DcMotorEx m : motors) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private boolean anyBusy() {
        for (DcMotorEx m : motors) if (m.isBusy()) return true;
        return false;
    }

    /** Turn in place to an absolute heading in degrees (CCW positive). Blocking. */
    public void turnToHeading(LinearOpMode opMode, double targetDeg, double maxTurnPower,
                              double timeoutSec) {
        ElapsedTime timer = new ElapsedTime();
        while (opMode.opModeIsActive() && timer.seconds() < timeoutSec) {
            double error = AngleUnit.normalizeDegrees(targetDeg - getHeadingDegrees());
            if (Math.abs(error) < 1.5) break;
            // Positive error = need to turn CCW; our "turn" input is + clockwise
            double turn = Range.clip(-error * TURN_KP, -maxTurnPower, maxTurnPower);
            // Minimum power so the robot doesn't stall short of the target
            if (Math.abs(turn) < 0.08) turn = Math.copySign(0.08, turn);
            setWheelPowers(0, 0, turn);
            opMode.telemetry.addData("Turning", "target %.1f, heading %.1f, error %.1f",
                    targetDeg, getHeadingDegrees(), error);
            opMode.telemetry.update();
        }
        stop();
    }

    // ---------------- Telemetry helper ----------------

    public String wheelPositions() {
        return String.format("FL %d  FR %d  BL %d  BR %d",
                frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(),
                backLeft.getCurrentPosition(), backRight.getCurrentPosition());
    }
}
