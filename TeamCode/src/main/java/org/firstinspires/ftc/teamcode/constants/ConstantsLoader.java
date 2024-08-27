package org.firstinspires.ftc.teamcode.constants;

import static org.firstinspires.ftc.teamcode.constants.Constants.FileConstants.*;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Scalar;

import java.io.*;
import java.lang.reflect.*;
import java.util.*;
import java.util.stream.*;

/**
 * <h1>Constants Loader</h1>
 * <p>
 *     The constants loader class attempts to override the values located in each static nested
 *     class found within the {@link Constants} using values found in text files corresponding names
 *     located at the following path:
 * </p>
 * <p>
 *     /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants/
 * </p>
 * <p>
 *     If a static nested class doesn't have a corresponding text file, it will be silently ignored.
 * </p>
 * <p>
 *     When the constants loader locates a file that corresponds with one of the nested constants
 *     classes, it will iterate through each field of the class and attempt to find a matching field
 *     in the file. In order to be recognized by the ConstantsLoader the field in the file must be
 *     formatted as follows:
 * </p>
 * <p>
 *     FIELD_NAME=VALUE
 * </p>
 * <p>
 *     One exception to this rule is Pose2D which must be formatted as follows:
 * </p>
 * <p>
 *     FIELD_NAME=Pose2D(X,Y,HEADING)
 * </p>
 * <p>
 *     The flavour of Pose2D (Roadrunner, SparkFunOTOS, etc) doesn't matter as the value in the text
 *     file will be assigned to the type of the field in the nested class.
 * </p>
 * <p>
 *     Any of the following conditions will cause the ConstantsLoader to skip overwriting a field:
 *     <ul>
 *         <li>The class field has no corresponding file field and vice-versa.</li>
 *         <li>The file field is not formatted as shown above.</li>
 *         <li>The file field and class field types mismatch.</li>
 *         <li>The class field is not static, not public, or final.</li>
 *     </ul>
 * </p>
 * <p>
 *     By default all failures to overwrite fields in the nested constants classes are ignored
 *     silently. However, in some instances displaying the reason can be useful if you are
 *     debugging. To enable this feature, call the constructor with the opModes telemetry object.
 * </p>
 */
public final class ConstantsLoader {
    private final boolean debug;
    @Nullable
    private final Debugger debugger;

    /**
     * Creates a new ConstantsLoader with debugging disabled. To enable debugging, pass the opModes
     * telemetry object to the constructor.
     */
    public ConstantsLoader() {
        this.debug    = false;
        this.debugger = null;
        MalformedPropertyException.debug = false;
    }

    /**
     * Creates a new constants loader with debugging enabled.
     * @param telemetry The telemetry to display debug information on
     */
    public ConstantsLoader(@NonNull Telemetry telemetry) {
        this.debug    = true;
        this.debugger = new Debugger(telemetry);
        MalformedPropertyException.debug = true;
    }


    @NonNull private List<File> getConstantsFilesFromOnbotJava() {
        File constantsDirectory = new File(CONSTANTS_FILE_LOCATION);

        if (constantsDirectory.isFile()) {
            if (!debug) return new ArrayList<>();

            String issue = "Failed To Load Constants File Directory"
                         + "\nReason: Conflicting File Constants in OnBot Java Directory."
                         + "\nHelp: Remove Directory Named \"Constants\"";

            debugger.addMessage(issue);

            return new ArrayList<>();
        }

        File[] constantsDirectoryFiles = constantsDirectory.listFiles();

        if (constantsDirectoryFiles == null) {
            if (!debug) return new ArrayList<>();

            String issue = "Failed To Load Constants Directory"
                         + "\nReason: No Files Were Found At The Specified Constants Directory";
            debugger.addMessage(issue);
            return new ArrayList<>();
        }

        List<File> textFiles = Arrays.stream(constantsDirectoryFiles)
                .filter(this::isTextFile)
                .collect(Collectors.toList());

        if (textFiles.isEmpty()) {
            String issue = "Failed To Load Constants Directory"
                         + "\nReason: No Files With Extension \".txt\" were found.";
            if (debug) debugger.addMessage(issue);
        }
        return textFiles;
    }

    private @NonNull Optional<Class<?>> matchConstantsClassToConstantsFile(
            @NonNull String fileName
    ) {
        for (Class<?> clazz : Constants.class.getClasses()) {
            if (!Modifier.isStatic(clazz.getModifiers())) continue;

            if (clazz.getSimpleName().equals(fileName)) return Optional.of(clazz);
        }
        String issue = "Failed To Match Constants Class With Name: " + fileName
                     + "\nNote: Non-Static Nested Classes Are Skipped";
        if (debug) debugger.addMessage(issue);
        return Optional.empty();
    }

    private void populateClassFromPropertiesFile(
            @NonNull Class<?> clazz,
            @NonNull String fileName
    ) {
        Properties properties = new Properties();

        try {
            properties.load(new FileInputStream(CONSTANTS_FILE_LOCATION + fileName));
            for (Field field : clazz.getFields()) { populateField(field, properties); }
        } catch (IOException ioException) {
            String issue = "Failed To Load Constants File " + fileName
                         + "\nReason: " + ioException.getMessage();
            if (debug) debugger.addMessage(issue);
        }
    }

    private void populateField(@NonNull Field field, @NonNull Properties properties) {
        String fieldName = field.getName();

        if (!properties.containsKey(fieldName) || !isLoadable(field)) return;

        try {
            switch (SupportedType.fieldToType(field)) {
                case FLOAT:
                    field.setFloat(fieldName, getFloatFromPropertiesFile(fieldName, properties));
                    break;
                case DOUBLE:
                    field.setDouble(fieldName, getDoubleFromPropertiesFile(fieldName, properties));
                    break;
                case BYTE:
                    field.setByte(fieldName, getByteFromPropertiesFile(fieldName, properties));
                    break;
                case SHORT:
                    field.setShort(fieldName, getShortFromPropertiesFile(fieldName, properties));
                    break;
                case INTEGER:
                    field.setInt(fieldName, getIntFromPropertiesFile(fieldName, properties));
                    break;
                case LONG:
                    field.setLong(fieldName, getLongFromPropertiesFile(fieldName, properties));
                    break;
                case BOOLEAN:
                    field.setBoolean(field, getBooleanFromPropertiesFile(fieldName, properties));
                    break;
                case CHAR:
                    field.setChar(field, getCharacterFromPropertiesFile(fieldName, properties));
                case STRING:
                    field.set(fieldName, getStringFromPropertiesFile(fieldName, properties));
                    break;
                case SERVO_DIRECTION:
                    try {
                        Servo.Direction servoDirection
                                = getServoDirectionFromPropertiesFile(fieldName, properties);
                        field.set(fieldName, servoDirection);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case MOTOR_DIRECTION:
                    try {
                        DcMotorSimple.Direction motorDirection
                                = getMotorDirectionFromPropertiesFile(fieldName, properties);
                        field.set(fieldName, motorDirection);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case ZERO_POWER_BEHAVIOUR:
                    try {
                        DcMotor.ZeroPowerBehavior zeroPowerBehavior
                                = getZeroPowerBehaviorFromPropertiesFile(fieldName, properties);
                        field.set(fieldName, zeroPowerBehavior);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case SPARK_FUN_POSE_2D:
                    try {
                        SparkFunOTOS.Pose2D pose2D
                                = getSparkFunPose2DFromPropertiesFile(fieldName, properties);
                        field.set(fieldName, pose2D);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case RUN_MODE:
                    try {
                        DcMotor.RunMode runMode
                                = getRunModeFromPropertiesFile(fieldName, properties);
                        field.set(fieldName, runMode);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case SCALAR:
                    try {
                        Scalar scalar
                                = getScalarFromPropertiesFile(fieldName, properties);
                        field.set(fieldName, scalar);
                    } catch (MalformedPropertyException ignored) { return; }
                    break;
                case UNSUPPORTED:
                    String message = "Failed To Load Field " + fieldName
                                   + "\nReason: Field Type Not Supported";
                    if (debug) debugger.addMessage(message);
                    break;
            }
        } catch (IllegalAccessException ignored) {}
    }

    /**
     * <p>
     *  Attempts to overwrite all of the values located in {@link Constants} with values found in
     *  corresponding text files located at the following path:
     * </p>
     * <p>
     *     /sdcard/FIRST/java/src/org/firstinspires/ftc/teamcode/Constants
     * </p>
     * <p>
     *     By default all errors are silently ignored. To enable debug mode, pass the opModes
     *     telemetry object into the constructor of the ConstantsLoader class.
     * </p>
     */
    public void load() {
        for (File file : getConstantsFilesFromOnbotJava()) {
            if (!isTextFile(file)) continue;

            String fileName = file.getName();

            Optional<Class<?>> constantsClass
                    = matchConstantsClassToConstantsFile(stripFileExtension(fileName));

            if (!constantsClass.isPresent()) continue;

            populateClassFromPropertiesFile(constantsClass.get(), fileName);
        }


    }

    private boolean isTextFile(@NonNull File file) {
        String fileName = file.getName();

        int extensionPosition = fileName.lastIndexOf('.') + 1;

        return fileName.substring(extensionPosition).equals("txt");
    }

    @NonNull private String stripFileExtension(@NonNull String fileName) {
        int extensionOffset = fileName.lastIndexOf('.');

        return fileName.substring(0, extensionOffset);
    }

    private float getFloatFromPropertiesFile(@NonNull String key, @NonNull Properties properties) {
        return Float.parseFloat(properties.getProperty(key));
    }

    private double getDoubleFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) {
        return Double.parseDouble(properties.getProperty(key));
    }

    private byte getByteFromPropertiesFile(@NonNull String key, @NonNull Properties properties) {
        return Byte.parseByte(properties.getProperty(key));
    }

    private short getShortFromPropertiesFile(@NonNull String key, @NonNull Properties properties) {
        return Short.parseShort(properties.getProperty(key));
    }

    private int getIntFromPropertiesFile(@NonNull String key, @NonNull Properties properties) {
        return Integer.parseInt(properties.getProperty(key));
    }

    private long getLongFromPropertiesFile(@NonNull String key, @NonNull Properties properties) {
        return Long.parseLong(properties.getProperty(key));
    }

    private boolean getBooleanFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) {
        return Boolean.parseBoolean(properties.getProperty(key));
    }

    private char getCharacterFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) {
       return properties.getProperty(key).toCharArray()[0];
    }

    @NonNull private String getStringFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) {
        return properties.getProperty(key);
    }

    @NonNull private Servo.Direction getServoDirectionFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String servoDirectionString = properties.getProperty(key);

        switch (servoDirectionString.toLowerCase()) {
            case "forward":
                return Servo.Direction.FORWARD;
            case "reverse":
            case "backwards":
                return Servo.Direction.REVERSE;
            default:
                throw new MalformedPropertyException(
                        servoDirectionString,
                        "Failed To Parse Direction",
                        key,
                        debugger
                );
        }
    }

    @NonNull private DcMotorSimple.Direction getMotorDirectionFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String motorDirectionString = properties.getProperty(key);

        switch (motorDirectionString.toLowerCase()) {
            case "forward":
                return DcMotor.Direction.FORWARD;
            case "reverse":
            case "backwards":
                return DcMotorSimple.Direction.REVERSE;
            default:
                throw new MalformedPropertyException(
                        motorDirectionString,
                        "Failed To Parse Direction",
                        key,
                        debugger
                );
        }
    }

    @NonNull private DcMotor.ZeroPowerBehavior getZeroPowerBehaviorFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String zeroPowerBehaviorString = properties.getProperty(key);

        switch (zeroPowerBehaviorString.toLowerCase()) {
            case "float":
                return DcMotor.ZeroPowerBehavior.FLOAT;
            case "brake":
                return DcMotor.ZeroPowerBehavior.BRAKE;
            case "unknown":
                return DcMotor.ZeroPowerBehavior.UNKNOWN;
            default:
                throw new MalformedPropertyException(
                        zeroPowerBehaviorString,
                        "Failed To Parse ZeroPowerBehavior",
                        key,
                        debugger
                );
        }
    }

    @NonNull private DcMotor.RunMode getRunModeFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String runModeString = properties.getProperty(key);

        switch (runModeString.toLowerCase()) {
            case "run_to_position":
               return DcMotor.RunMode.RUN_TO_POSITION;
            case "run_using_encoders":
                return DcMotor.RunMode.RUN_USING_ENCODER;
            case "run_without_encoders":
                return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            case "stop_and_reset_encoders":
                return DcMotor.RunMode.STOP_AND_RESET_ENCODER;
            default:
                throw new MalformedPropertyException(
                        runModeString,
                        "Failed To Parse As RunMode",
                        "RunMode",
                        debugger
                );
        }
    }

    @NonNull private SparkFunOTOS.Pose2D getSparkFunPose2DFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String poseString = properties.getProperty(key);

        double[] poseValues;

        try {
            poseValues = parseThreePartValue(poseString);
        } catch (NumberFormatException numberFormatException) {
            throw new MalformedPropertyException(
                   poseString,
                   numberFormatException.getMessage(),
                   key,
                   debugger
            );
        }

        return new SparkFunOTOS.Pose2D(poseValues[0], poseValues[1], poseValues[2]);
    }

    @NonNull private Scalar getScalarFromPropertiesFile(
            @NonNull String key,
            @NonNull Properties properties
    ) throws MalformedPropertyException {
        String scalarString = properties.getProperty(key);

        double[] scalarValues;

        try {
            scalarValues = parseThreePartValue(scalarString);
        } catch (NumberFormatException numberFormatException) {
            throw new MalformedPropertyException(
                    scalarString,
                    numberFormatException.getMessage(),
                    key,
                    debugger
            );
        }

        return new Scalar((int) scalarValues[0], (int) scalarValues[1], (int) scalarValues[2]);
    }

    @NonNull private double[] parseThreePartValue(
            @NonNull String value
    ) throws NumberFormatException {
        int indexOfFirstOpenParentheses   = value.indexOf('(');
        int indexOfFirstClosedParentheses = value.indexOf(')');
        int indexOfFirstComma             = value.indexOf(',');
        int indexOfSecondComma            = value.indexOf(',', indexOfFirstComma + 1);

        String valueOneString
                = value.substring(indexOfFirstOpenParentheses + 1, indexOfFirstComma);
        String valueTwoString
                = value.substring(indexOfFirstComma + 1, indexOfSecondComma + 1);
        String valueThreeString
                = value.substring(indexOfSecondComma + 1, indexOfFirstClosedParentheses);

        double valueOne   = Double.parseDouble(valueOneString);
        double valueTwo   = Double.parseDouble(valueTwoString);
        double valueThree = Double.parseDouble(valueThreeString);

        return new double[]{valueOne, valueTwo, valueThree};
    }

    private boolean isLoadable(@NonNull Field field) {
        int modifiers    = field.getModifiers();
        String fieldName = field.getName();

        boolean fieldIsPublic = Modifier.isPublic(modifiers);
        boolean fieldIsStatic = Modifier.isStatic(modifiers);
        boolean fieldIsFinal  = Modifier.isFinal(modifiers);

        if (!fieldIsPublic || !fieldIsStatic || fieldIsFinal) {
            String message = "Field " + fieldName + " cannot be loaded";

            if (!fieldIsPublic) message += "\nReason: Field Is Not Public";
            if (!fieldIsStatic) message += "\nReason: Field Is No Static";
            if (fieldIsFinal)   message += "\nReason: Field Is Final";

            if (debug) debugger.addMessage(message);
            return false;
        }
        return true;
    }

    private static class MalformedPropertyException extends Exception {
        public static boolean debug = false;

        public MalformedPropertyException(
                @NonNull String value,
                @NonNull String reason,
                @NonNull String name,
                @NonNull Debugger debugger
        ) {
            super("Property " + name + " Cannot Be Loaded");

            if (!debug) return;

            String issue = "Field " + name + " Cannot Be Loaded"
                         + "\nReason: Value [" + value + "] Is Malformed | " + reason;

            debugger.addMessage(issue);
        }
    }

    private static class Debugger {
        private final ArrayList<String> output;
        private final Telemetry telemetry;

        public Debugger(@NonNull Telemetry telemetry) {
            this.output    = new ArrayList<>();
            this.telemetry = telemetry;
        }

        public void addMessage(@NonNull String message) {
            output.add(message);
        }

        public void display() {
            telemetry.addData("Number Of Issues", output.size());

            for (String message: output) { telemetry.addLine("\n" + message); }
        }
    }
}
