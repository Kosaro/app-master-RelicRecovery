package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by okosa on 9/9/2017.
 */

public class Hardware {
    //Configuration Constants
    private static final String LEFT_MOTOR = "lm";
    private static final String RIGHT_MOTOR = "rm";
    private static final String JEWEL_SERVO = "js";
    private static final String COLOR_SENSOR = "cs";

    public static final double JEWEL_SERVO_DOWN = 0;
    public static final double JEWEL_SERVO_UP = 1;

    //Hardware Devices
    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo jewelServo;

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
        leftMotor = getHardwareDevice(DcMotor.class, LEFT_MOTOR);
        rightMotor = getHardwareDevice(DcMotor.class, RIGHT_MOTOR);
        jewelServo = getHardwareDevice(Servo.class, JEWEL_SERVO);
        colorSensor = hardwareMap.i2cDevice.get(COLOR_SENSOR);
        colorReader = new I2cDeviceSynchImpl(colorSensor, I2cAddr.create8bit(0x3c), false);

        colorReader.engage();
        colorReader.write8(3, 0);
    }


    public void setDriveTrainRunMode(DcMotor.RunMode runMode) {
        leftMotor.setMode(runMode);
        rightMotor.setMode(runMode);
    }

    Hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
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
