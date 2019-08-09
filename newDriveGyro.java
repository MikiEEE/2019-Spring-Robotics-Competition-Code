
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


public class DriveGyro  {

    static final double     COUNTS_PER_MOTOR_REV    = 288 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 90*(1/25.4) ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     REV_SPEED               = 0.8;
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    public DcMotor  left   = null;
    public DcMotor  right  = null;

    // The IMU sensor object

    BNO055IMU imu;

    ElapsedTime mClock = new ElapsedTime();
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

            /* Declare OpMode members. */
    HardwarePushbot        robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    ModernRoboticsI2cGyro   gyro    = null;                    // Additional Gyro device
    public static IMUInfo imuTest;


    public static HardwareMap hardwareMap;
    public Telemetry telemetry;
    public LinearOpMode opmode;

    public  DriveGyro(HardwareMap hardware, LinearOpMode opmodeRun ) {
        hardwareMap = hardware;
        telemetry = opmodeRun.telemetry;
        opmode = opmodeRun;
        init();
    }

    public void gyroDrive ( double speed, double distance,double angle) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active

        // Determine new target position, and pass to motor controller
        moveCounts = (int)(distance * COUNTS_PER_INCH);
        newLeftTarget = robot.left.getCurrentPosition() + moveCounts;
        newRightTarget = robot.right.getCurrentPosition() - moveCounts;

        // Set Target and Turn On RUN_TO_POSITION
        robot.left.setTargetPosition(newLeftTarget);
        robot.right.setTargetPosition(newRightTarget);

        robot.left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion.
        speed = Range.clip(Math.abs(speed), 0.0, 1.0);
        robot.left.setPower(speed);
        robot.right.setPower(speed);

        // keep looping while we are still active, and BOTH motors are running.
        while ((robot.left.isBusy() && robot.right.isBusy())) {

            // adjust relative speed based on heading error.
            error = getError(angle);
            steer = getSteer(error, P_DRIVE_COEFF);

            // if driving in reverse, the motor correction also needs to be reversed
            if (distance < 0)
                steer *= -1.0;

            leftSpeed = speed - steer;
            rightSpeed = speed + steer;

            // Normalize speeds if either one exceeds +/- 1.0;
            max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
            if (max > 1.0)
            {
                leftSpeed /= max;
                rightSpeed /= max;
            }

            robot.left.setPower(leftSpeed);
            robot.right.setPower(rightSpeed);
            }

            // Stop all motion;
            robot.left.setPower(0);
            robot.right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveMe(double speed, double distance, double angle) {
        gyroDrive(speed, -distance, angle);
        return;
    }

        /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     */
    public void turnMe(double turn, double speed, double tolerance) {
        double target = formatTarget(turn, imuTest.getAngle());
        boolean isCorrect = false;
        double error;
        stop();
        do{
            do{
                error =  getTurnError(target);
                if( error  < 0 ) {
                        turnRight(speed);
                }
                else{
                        turnLeft(speed);
                }
            }while(Math.abs(error) > tolerance);

            stop();

            error =  getTurnError(target);
            if(Math.abs(error) < tolerance){
                isCorrect = true;
            }
        }while(!isCorrect);
        return;
    }

    public boolean getDirection(double target,double current) {
        if (target > current) {
            return false;
        }
        else {
            return true;
        }
    }

    public void init(){
        /*
         * Initialize the standard drive system variables.
         * The init() method of the hardware class does most of the work here
         */
         imuTest = new IMUInfo(hardwareMap);
         robot.init(hardwareMap);
         gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
         robot.initGyro();
          while(robot.imu.isGyroCalibrated()==false){
              continue;
         }
         Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
         robot.left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         robot.right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         gyro.calibrate();
         // make sure the gyro is calibrated before continuing
         while (gyro.isCalibrating())  {
            continue;
         }
         robot.left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         robot.right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         gyro.resetZAxisIntegrator();
         telemetry.addData(":","DriveMeInitialized");
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public double formatTarget(double turn, double currentAngle) {
        double target = 0;
        double addedAngle = turn + currentAngle;
        double remainder = 0;
        int compare = (int)Math.round(addedAngle);
        if(compare > 180) {
            remainder = currentAngle - 180;
            target = remainder + turn - 180;
        }
        else{
            target = addedAngle;
        }

        return target;
    }

    public void turnLeft(double speed) {
        robot.left.setPower(speed);
        robot.right.setPower(speed);
        return;
    }

    public void turnRight(double speed) {
        robot.left.setPower(-speed);
        robot.right.setPower(-speed);
        return;
    }

    public void stop() {
        robot.left.setPower(0.0);
        robot.right.setPower(0.0);
        return;
    }

    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - gyro.getIntegratedZValue();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    public double getTurnError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - imuTest.getAngle();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }
    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
}
