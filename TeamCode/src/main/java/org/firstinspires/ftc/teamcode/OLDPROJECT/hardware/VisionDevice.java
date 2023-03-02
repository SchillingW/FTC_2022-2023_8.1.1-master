package org.firstinspires.ftc.teamcode.OLDPROJECT.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class VisionDevice {

    public static final int viewSize = 5;

    public ElapsedTime timer = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "StaticDischarge.tflite";

    private static final String[] LABELS = {
            "Balloon",
            "Gears",
            "Outlet",
    };
//
    private static final String VUFORIA_KEY =
            "Acdz4y//////AAABmW70zDxKuEnWrJY6iYczknpDiqeSYqA9IRXuJzhbM0+fRsY0g5rvwouqXOHVlvH/Nf4497j/5YltWntYrjROZWpFeh6E3RbeYfTKzmBCugWJgep4koejh5vMsEAouaqEqQA53H89VYjlf5uEA8Z9p0Ti3FC4yP+fGy68ktKx22IVuprZr9nwfDv+ky2RBfL+FP42Ew+yqTVguX+NQ//41Fv9XXZxUaL0ZL4FnZxzb6A9KlXPvIlN41QpAYAT/n3XhuD8lwdVS2bcjPgaFD1qbcsVKg8V57QGwfUdd9CfEHqwJatdqEOamTqQfAf4wdnrs7TWDwBkpmErMbir+yPMImtlumeXdQezlnkUG9V4MBXy";


    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    public VisionDevice(Telemetry telemetry, HardwareMap map) {

        this.telemetry = telemetry;
        this.hardwareMap = map;
    }


    public void init() {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.5, 8.0/6.0);
        }
    }

    public int perform(float xPosition) {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Objects Detected", updatedRecognitions.size());
                // step through the list of recognitions and display image position/size information for each one
                // Note: "Image number" refers to the randomized image orientation/number
                for (Recognition recognition : updatedRecognitions) {
                    double col = (recognition.getLeft() + recognition.getRight()) / 2;
                    double row = (recognition.getTop() + recognition.getBottom()) / 2;
                    double width = Math.abs(recognition.getRight() - recognition.getLeft());
                    double height = Math.abs(recognition.getTop() - recognition.getBottom());

                    telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                    telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                    if (recognition.getLabel().equals("Outlet")) {
                        telemetry.addData("Real Image Outlet", 0);
                        return 0;

                    }
                    if (recognition.getLabel().equals("Gears")) {
                        telemetry.addData("Real Image Gear", 1);
                        return 1;
                    }
                    if (recognition.getLabel().equals("Balloon")) {
                        telemetry.addData("Real Image Balloon", 2);
                        return 2;
                    }
                }
            }
        }
        return -1;
    }




    //Detect and add if statements for which path to take (path1, path2, path3)


    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }
}
