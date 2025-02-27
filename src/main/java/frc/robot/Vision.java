package frc.robot;

import java.util.HashSet;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Timer;

public class Vision {
    private Thread m_visionThread;
    private int target = 0;
    private float where = Float.NaN;

    /**
     * What target do I look for?
     * send 0 to turn me off.
     * 
     * @param targetId The target ID to look for
     */
    public void lookFor(int targetId) {
        this.target = targetId;
    }

    public boolean isTargetInSight() {
        return !Float.isNaN(where);
    }

    /**
     * 
     * @return Location of target. 0 Dead ahead, -1 way left, +1 way right.
     */
    public float targetPosition() {
        return where;
    }

    public synchronized void start() {
        if (m_visionThread == null) {
            visionInit();
        }
    }

    public synchronized void stop() {
        if (m_visionThread != null) {
            m_visionThread.interrupt();
            m_visionThread = null;
        }
    }

    private void visionInit() {
        // Only vision in robotInit below here
        m_visionThread = new Thread(
                () -> {
                    var camera = CameraServer.startAutomaticCapture("raw", 0);

                    var cameraWidth = 800;
                    var cameraHeight = 600;

                    camera.setResolution(cameraWidth, cameraHeight);

                    var cvSink = CameraServer.getVideo("raw");
                    var outputStream = CameraServer.putVideo("RioApriltags", cameraWidth, cameraHeight);

                    var mat = new Mat();
                    var grayMat = new Mat();

                    var pt0 = new Point();
                    var pt1 = new Point();
                    var pt2 = new Point();
                    var pt3 = new Point();
                    var center = new Point();
                    var red = new Scalar(0, 0, 255);
                    var green = new Scalar(0, 255, 0);

                    var aprilTagDetector = new AprilTagDetector();

                    var config = aprilTagDetector.getConfig();
                    //config.quadSigma = 0.8f;


                    aprilTagDetector.setConfig(config);

                   // var quadThreshParams = aprilTagDetector.getQuadThresholdParameters();
                   // quadThreshParams.minClusterPixels = 250;
                  //  quadThreshParams.criticalAngle *= 5; // default is 10
                   // quadThreshParams.maxLineFitMSE *= 1.5;
                   // aprilTagDetector.setQuadThresholdParameters(quadThreshParams);

                    aprilTagDetector.addFamily("tag36h11", 1);

                    var timer = new Timer();
                    timer.start();
                    var count = 0;

                    while (!Thread.interrupted() && m_visionThread != null) {
                        if (cvSink.grabFrame(mat) == 0) {
                            outputStream.notifyError(cvSink.getError());
                            continue;
                        }

                        Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);
                        Imgproc.equalizeHist(grayMat, grayMat);

                        var results = aprilTagDetector.detect(grayMat);

                        var set = new HashSet<>();

                        float w = Float.NaN;

                        for (var result : results) {
                            if (result.getId() == target) {
                                w = (float) (((results[0].getCenterX() - (cameraWidth / 2)) / cameraWidth));
                            }
                            count += 1;
                            pt0.x = result.getCornerX(0);
                            pt1.x = result.getCornerX(1);
                            pt2.x = result.getCornerX(2);
                            pt3.x = result.getCornerX(3);

                            pt0.y = result.getCornerY(0);
                            pt1.y = result.getCornerY(1);
                            pt2.y = result.getCornerY(2);
                            pt3.y = result.getCornerY(3);

                            center.x = result.getCenterX();
                            center.y = result.getCenterY();

                            set.add(result.getId());

                            Imgproc.line(mat, pt0, pt1, red, 5);
                            Imgproc.line(mat, pt1, pt2, red, 5);
                            Imgproc.line(mat, pt2, pt3, red, 5);
                            Imgproc.line(mat, pt3, pt0, red, 5);

                            Imgproc.circle(mat, center, 4, green);
                            Imgproc.putText(mat, String.valueOf(result.getId()), pt2, Imgproc.FONT_HERSHEY_SIMPLEX, 2,
                                    green, 7);

                        }

                        where = w;

                        for (var id : set) {
                            System.out.println("Tag: " + String.valueOf(id));
                        }

                        if (timer.advanceIfElapsed(1.0)) {
                            System.out.println("detections per second: " + String.valueOf(count));
                            count = 0;
                        }

                        outputStream.putFrame(mat);
                    }
                    aprilTagDetector.close();
                });
        m_visionThread.setDaemon(true);
        m_visionThread.start();
    }

}
