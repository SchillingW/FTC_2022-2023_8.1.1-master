package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

// vision device with camera
public class VisionDevice {

    // store telemetry device
    public Telemetry tele;

    // model config
    public static final String VUFORIA_KEY = "Acdz4y//////AAABmW70zDxKuEnWrJY6iYczknpDiqeSYqA9IRXuJzhbM0+fRsY0g5rvwouqXOHVlvH/Nf4497j/5YltWntYrjROZWpFeh6E3RbeYfTKzmBCugWJgep4koejh5vMsEAouaqEqQA53H89VYjlf5uEA8Z9p0Ti3FC4yP+fGy68ktKx22IVuprZr9nwfDv+ky2RBfL+FP42Ew+yqTVguX+NQ//41Fv9XXZxUaL0ZL4FnZxzb6A9KlXPvIlN41QpAYAT/n3XhuD8lwdVS2bcjPgaFD1qbcsVKg8V57QGwfUdd9CfEHqwJatdqEOamTqQfAf4wdnrs7TWDwBkpmErMbir+yPMImtlumeXdQezlnkUG9V4MBXy";
    public static final String TFOD_MODEL_ASSET = "StaticDischarge.tflite";
    public static final String[] LABELS = {"Balloon", "Gears", "Outlet"};

    // vision devices
    public VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    // vision result
    public int result;

    // initialize vision device
    public VisionDevice(Telemetry tele, HardwareMap map) {

        this.tele = tele;

        initVuforia(map);
        initTfod(map);
    }

    // perform vision and store result
    public void loop() {

        // store if new recognition
        int next = perform();
        if (next != -1) result = next;

        // telemetry debugging
        tele.addData("vision", result);
    }

    // perform vision and return result
    public int perform() {

        // get recognitions from tensorflow
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        // return recognition
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals("Outlet")) return 0;
                if (recognition.getLabel().equals("Gears")) return 1;
                if (recognition.getLabel().equals("Balloon")) return 2;
            }
        }

        // return no recognition
        return -1;
    }

    // initialize camera vuforia
    private void initVuforia(HardwareMap map) {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = map.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    // initialize vision tensorflow
    private void initTfod(HardwareMap map) {

        int tfodMonitorViewId = map.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", map.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        tfod.activate();
        tfod.setZoom(1.5, 16.0/9.0);
    }
}
