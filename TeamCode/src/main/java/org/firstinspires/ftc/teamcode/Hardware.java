package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by okosa on 9/9/2017.
 */

public class Hardware {
    //Configuration Constants
    private static final String LEFT_FRONT_MOTOR = "lf";
    private static final String RIGHT_FRONT_MOTOR = "rf";
    private static final String LEFT_REAR_MOTOR = "lr";
    private static final String RIGHT_REAR_MOTOR = "rr";
    private static final String JEWEL_SERVO = "js";
    private static final String COLOR_SENSOR = "cs";
    final static String IMU = "imu";

    private static final DcMotor.Direction LEFT_FRONT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    private static final DcMotor.Direction RIGHT_FRONT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotor.Direction LEFT_REAR_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
    private static final DcMotor.Direction RIGHT_REAR_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static final double JEWEL_SERVO_DOWN = 0;
    public static final double JEWEL_SERVO_UP = 1;

    //Hardware Devices
    DcMotor leftFrontMotor;
    DcMotor rightFrontMotor;
    DcMotor leftRearMotor;
    DcMotor rightRearMotor;
    Servo jewelServo;
    BNO055IMU imu;

    private byte[] colorCache;
    private I2cDevice colorSensor;
    private I2cDeviceSynch colorReader;

    //Local Variables
    private HardwareMap hardwareMap;
    private VuforiaLocalizer vuforia;
    private VuforiaTrackable relicTemplate;

    enum ColorDetected {
        BLUE("Blue"),
        RED("Red"),
        NONE("None");

        String description;

        public String toString() {
            return description;
        }

        ColorDetected(String description) {
            this.description = description;
        }
    }

    private void initialize() {
        leftFrontMotor = getHardwareDevice(DcMotor.class, LEFT_FRONT_MOTOR);
        rightRearMotor = getHardwareDevice(DcMotor.class, RIGHT_REAR_MOTOR);
        leftRearMotor = getHardwareDevice(DcMotor.class, LEFT_REAR_MOTOR);
        rightFrontMotor = getHardwareDevice(DcMotor.class, RIGHT_FRONT_MOTOR);
        jewelServo = getHardwareDevice(Servo.class, JEWEL_SERVO);
        colorSensor = getHardwareDevice(I2cDevice.class, COLOR_SENSOR);
        colorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);
        imu = getHardwareDevice(BNO055IMU.class, IMU);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        leftFrontMotor.setDirection(LEFT_FRONT_MOTOR_DIRECTION);
        rightFrontMotor.setDirection(RIGHT_FRONT_MOTOR_DIRECTION);
        leftRearMotor.setDirection(LEFT_REAR_MOTOR_DIRECTION);
        rightRearMotor.setDirection(RIGHT_REAR_MOTOR_DIRECTION);

        colorReader.engage();
        colorReader.write8(3, 0);
    }

    //Initialize IMU
    private void initializeImuParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }


    public void setDriveTrainRunMode(DcMotor.RunMode runMode) {
        leftFrontMotor.setMode(runMode);
        rightFrontMotor.setMode(runMode);
        leftRearMotor.setMode(runMode);
        rightRearMotor.setMode(runMode);
    }

    private void drive(double forwardValue, double sideValue, double rotationValue) {
        double leftFrontPower;
        double leftRearPower;
        double rightFrontPower;
        double rightRearPower;

        leftFrontPower = forwardValue + sideValue + rotationValue;
        leftRearPower = forwardValue - sideValue + rotationValue;
        rightFrontPower = forwardValue - sideValue - rotationValue;
        rightRearPower = forwardValue + sideValue - rotationValue;

        double max = Double.MIN_VALUE;
        if (Math.abs(leftFrontPower) > max)
            max = Math.abs(leftFrontPower);
        if (Math.abs(leftRearPower) > max)
            max = Math.abs(leftRearPower);
        if (Math.abs(rightFrontPower) > max)
            max = Math.abs(rightFrontPower);
        if (Math.abs(rightRearPower) > max)
            max = Math.abs(rightRearPower);
        if (max > 1) {
            leftFrontPower /= max;
            leftRearPower /= max;
            rightFrontPower /= max;
            rightRearPower /= max;
        }

        leftFrontMotor.setPower(leftFrontPower);
        leftRearMotor.setPower(leftRearPower);
        rightFrontMotor.setPower(rightFrontPower);
        rightRearMotor.setPower(rightRearPower);
    }

    public double turn(double finalHeading) {
        while (finalHeading < -180){
            finalHeading += 360;
        }
        while (finalHeading > 180){
            finalHeading -= 360;
        }

        double heading = getAngle();


        if (heading > 180) { // convert 0 - 360 range of heading to -180 - 180
            heading += 180;
            heading %= 360;
            heading -= 180;
        }

        double turnPower;
        if (Math.abs(heading - finalHeading) > 120) {
            turnPower = 1;
        } else if (Math.abs(heading - finalHeading) > 90) {
            turnPower = .7;
        } else if (Math.abs(heading - finalHeading) > 40) {
            turnPower = .4;
        } else if (Math.abs(heading - finalHeading) > 30) {
            turnPower = .35;
        } else if (Math.abs(heading - finalHeading) > 25) {
            turnPower = .3;
        } else if (Math.abs(heading - finalHeading) > 20) {
            turnPower = .2;

        } else if (Math.abs(heading - finalHeading) > 15) {
            turnPower = .2;

        } else if (Math.abs(heading - finalHeading) > 10) {
            turnPower = .15;
        } else if (Math.abs(heading - finalHeading) > 5) {
            turnPower = .15;
        } else {
            turnPower = .10;
        }

        if (Math.abs(heading - finalHeading) < 3
                ) {
            return 0;
        }

        if (heading <= 180 && heading >= 0 && finalHeading <= 180 && finalHeading >= 0) {
            if (finalHeading > heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading >= -180 && heading <= 0 && finalHeading >= -180 && finalHeading <= 0) {
            if (finalHeading > heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading >= -180 && heading <= 0 && finalHeading <= 180 && finalHeading >= 0) {
            if (finalHeading - heading < (heading + 360) - finalHeading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        } else if (heading <= 180 && heading >= 0 && finalHeading >= -180 && finalHeading <= 0) {
            if (heading - finalHeading > (finalHeading + 360) - heading) {
                return turnPower;
            } else {
                return -turnPower;
            }
        }
        return .00005;
    }

    //Find angle from gyro
    double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                .toAngleUnit(AngleUnit.DEGREES).firstAngle;
    }

    Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
        initializeImuParameters();
    }

    private <T> T getHardwareDevice(Class<? extends T> classOrInterface, String deviceName) {
        try {
            return hardwareMap.get(classOrInterface, deviceName);
        } catch (Exception e) {
            RobotLog.e("Missing " + classOrInterface.toString() + " " + deviceName);
            return null;
        }
    }

    public void initializeVuforia() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdYQ1UT/////AAAAGduICslDnEnttzNkRGI2spxKjPBtdo/7cWrldv0MqHAmbK0Fyjw65zsW4JCkN6GRGjkwkLLX4kMkfjY2j/7K9K74AA1RRn1QaxpqfHqWfPXudWPzt4Y3PaLHK5c6ge6m6PyDYTZMxZmgb2jS5JaR0KPUf8Vmu1ysEOZfSNcSC20G56QRO/9VpJRrfetFMlsDiAwmsj+muYdKN5fwRCW3N8KK7CVD2ad9mXKvv45082O9PL0zXxq1vYPeKmn/27V1ihKOI0JHL5vEIeN3XeA56SM1f7yiLk2LFmkY+sM6K+HaDL+wLIulcuUIidqZ0xwKFFHCPjssVaZ25RtHYUY4nIvS+LJdzO+FdTYNDqtOn95Q";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    public RelicRecoveryVuMark getPictograph() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public ColorDetected getColorDetected(){
        colorCache = colorReader.read(0x04, 1);
         if (colorCache[0]>1 && colorCache[0]<4){
             return ColorDetected.BLUE;
         }
         if (colorCache[0]>9 && colorCache[0]<12){
             return ColorDetected.RED;
         }
         return ColorDetected.NONE;
    }
}
