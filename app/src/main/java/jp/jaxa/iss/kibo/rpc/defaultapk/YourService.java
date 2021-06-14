package jp.jaxa.iss.kibo.rpc.defaultapk;

//
import android.graphics.Bitmap;
import android.util.Log;
//
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
//
import gov.nasa.arc.astrobee.types.*;
//
import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.ResultPoint;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.detector.FinderPattern;
//
import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
//
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;


public class YourService extends KiboRpcService {

    @Override
    protected void runPlan1() {
        api.startMission();

        final Quaternion q = new Quaternion(-0.11316165328025818f ,-0.12624533474445343f ,-0.6577022671699524f ,0.7337453961372375f);

        move_to(11.2100, -9.8000, 4.7900, q);

        final String qrContents = qr_read();
        final float qrData[] = interpretQRString(qrContents); // {kozPattern, x, y, z}
        Log.i("Interpret QR String", Arrays.toString(qrData));

        final int pattern = (int)qrData[0];

        if (pattern == 7) {
            move_pattern7(qrData[1], qrData[3]);
        }
        else if (pattern == 5 || pattern == 6) {
            move_pattern56(qrData[1] - 0.05f, qrData[3]);
        }
        else {
            move_to(qrData[1], qrData[2], qrData[3], new Quaternion(0, 0, -0.707f, 0.707f));
        }

        final Point AR_Center_3d = ar_read();
//        final Point laserPoint = new Point(1.0f, 0.0f, 0.0f);
//        Quaternion quaLaserToTarget = quaternionFrom2Points(laserPoint, AR_Center_3d);
        double angleToLaser[] = getLaserAngleToTarget(AR_Center_3d);
        Quaternion qToTarget = eulerAngleToQuaternion(angleToLaser[0], 0.0f, angleToLaser[1]);
        final Quaternion qToTurn = combineQuaternion(qToTarget, new Quaternion(0, 0, -0.707f, 0.707f));

//        final Mat undistortAr = undistortPoints(AR_Center);
//        final double[] laser = {711, 455};
//        final double[] angleToTurn = pixelDistanceToAngle(undistortAr.get(0, 0), laser, pattern);
//        final Quaternion imageQ = eulerAngleToQuaternion(angleToTurn[1], 0, angleToTurn[0]);

        move_to(qrData[1], qrData[2], qrData[3], qToTurn);

        api.laserControl(true);
        api.takeSnapshot();
        api.laserControl(false);

        move_toB(pattern, qrData[1], qrData[3]);

        boolean cleared;
        do {
            cleared = api.reportMissionCompletion();
        } while (!cleared);
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    // GLOBAL CONSTANT

    final int LOOP_MAX = 3;
    /*
    final int NAV_MAX_WIDTH = 1280;
    final int NAV_MAX_HEIGHT = 960;
    */

    final double[] CAM_MATSIM = {
            567.229305, 0.0, 659.077221,
            0.0, 574.192915, 517.007571,
            0.0, 0.0, 1.0
    };

    final double[] DIST_COEFFSIM = {
            -0.216247, 0.03875, -0.010157, 0.001969, 0.0
    };

    final float MARKER_LENGTH = 0.05f;

    class imagePoint {
        float x, y;
        imagePoint(float x, float y) {
            this.x = x;
            this.y = y;
        }

        String dump() {
            return ("[" + x + ", " + y + "]");
        }
    }

    // UTILITIES

    private void log_kinematics() {
        final String TAG = "log_position";
        Point p = api.getTrustedRobotKinematics().getPosition();
        Quaternion q = api.getTrustedRobotKinematics().getOrientation();
        Log.i(TAG, "Position= " + p.getX() + ", " + p.getY() + ", " + p.getZ());
        Log.i(TAG, "Orientation= " + q.toString());
    }

    // MOVING

    private void move_to(double x, double y, double z, Quaternion q) {
        final String TAG = "move_to";
        final Point p = new Point(x, y, z);

        int counter = 0;
        Result result;

        Log.i(TAG, "Start");

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        // info
        //log_kinematics();
        Log.i(TAG, "Done");
    }

    private void move_to(Point p, Quaternion q) {
        final String TAG = "move_to";
        int counter = 0;
        Result result;

        Log.i(TAG, "Start");

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        // info
        //log_kinematics();
        Log.i(TAG, "Done");
    }

    // QR CODE READING

    private BinaryBitmap getNavImg() {
        final String TAG = "getNavImg";
        long start = System.currentTimeMillis();

        // img processing shit
        Log.i(TAG, "Processing img");

        Bitmap bMap = Bitmap.createBitmap(api.getBitmapNavCam(), 526, 378, 250, 250);
        int[] intArr = new int[bMap.getWidth() * bMap.getHeight()];
        bMap.getPixels(intArr, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        final LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArr);
        final BinaryBitmap out = new BinaryBitmap(new HybridBinarizer(source));

        long end = System.currentTimeMillis();
        Log.i(TAG, "ElapsedTime=" + (end-start));
        return out;
    }

    private String QRreader() {
        final String TAG = "QRreader";
        Log.i(TAG, "Start");

        final Map<DecodeHintType, Object> hints = new Hashtable<>();
        final List<BarcodeFormat> qr = new ArrayList<>(); qr.add(BarcodeFormat.QR_CODE);
        hints.put(DecodeHintType.POSSIBLE_FORMATS, qr);
        hints.put(DecodeHintType.CHARACTER_SET, "utf-8");

        try {
            Thread.currentThread().sleep(15000);
        }
        catch (Exception e) {
            e.printStackTrace();
        }

        String text = null;
        int counter = 0;
        while (text == null && counter < LOOP_MAX) {
            try {
                long start = System.currentTimeMillis();
                final BinaryBitmap bitmap = getNavImg();
                final com.google.zxing.Result data = new QRCodeReader().decode(bitmap, hints);

                final ResultPoint[] p = data.getResultPoints();
                for (int i = 0; i < p.length; i++) {
                    Log.i(TAG, "ResultPoint[" + i + "]=" + p[i].toString());
                    if (p[i] instanceof FinderPattern) {
                        Log.i(TAG, "estimatedModuleSize = " + ((FinderPattern) p[i]).getEstimatedModuleSize());
                    }
                }

                text = data.getText();
                long end = System.currentTimeMillis();
                Log.i(TAG, "ElapsedTime[" + counter + "]=" + (end-start));
                return text;
            }
            catch (Exception e) {
                e.printStackTrace();
            }
            counter++;
        }

        Log.i(TAG, "Failed");
        return null;
    }

    private String qr_read() {
        final String TAG = "qr_read";
        long start = System.currentTimeMillis();

        // qr code reading
        Log.i(TAG, "Reading qr code");
        String text = QRreader();
        api.sendDiscoveredQR(text);

        Log.i(TAG, "Data = " + text);
        Log.i(TAG, "Done -- Success");

        long end = System.currentTimeMillis();
        Log.i(TAG, "qr_read time :" + (end - start));
        return text;
    }

    private float[] interpretQRString(String qrContents) {
        Log.i("Interpret QR String", "Start");
        String[] multi_contents = qrContents.split(",");

        int pattern = Integer.parseInt(multi_contents[0].substring(5));

        float final_x = Float.parseFloat(multi_contents[1].substring(4));
        float final_y = Float.parseFloat(multi_contents[2].substring(4));
        float final_z = Float.parseFloat(multi_contents[3].substring(4, multi_contents[3].length()-1));
        Log.i("Interpret QR String", "Done");
        return new float[] {pattern, final_x, final_y, final_z};
    }

    // AR CODE READING AND RELATED MATHS OPERATION

    private Point ar_read() {
        final String TAG = "ar_read";
        final long start = System.currentTimeMillis();

        final Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        try {
            Thread.currentThread().sleep(21000);
        } catch (Exception e) {
            e.printStackTrace();
        }

        int counter = 0;
        Log.i(TAG, "Reading AR");
        while (ids.rows() != 4 && counter < LOOP_MAX) {
            Mat pic = new Mat(api.getMatNavCam(), new Rect(320, 240, 640, 500));
            Aruco.detectMarkers(pic, dict, corners, ids);
            counter++;
        }

        // LOG
        for (int j = 0; j < corners.size(); j++) {
            Mat corner = corners.get(j);

            for (int z = 0; z < corner.cols(); z++) {
                double[] coord = corner.get(0, z);
                coord[0] = coord[0] + 320;
                coord[1] = coord[1] + 240;
                corner.put(0, z, coord);
            }
            corner.copyTo(corners.get(j));
            Log.i(TAG, "corner=" + corner.dump());
            Log.i(TAG, "corners[" + j + "]=" + corners.get(j).dump());
        }

        Log.i(TAG, "ids= " + ids.dump());
        long end = System.currentTimeMillis();
        Log.i(TAG, "ar_read_time=" + (end-start));
        //

        imagePoint[] markersCenter = new imagePoint[4];

        if(ids.rows() == 4) {
            Log.i(TAG, "All 4 ids are found.");
            for (int i = 0; i < 4; i++) {
                markersCenter[i] = findCenterRect(corners.get(i));
                Log.i(TAG, "Marker Center[" + i + "](id: " + ids.get(i, 0)[0] + ")=" + markersCenter[i].dump());
            }

        } else {
            Log.i(TAG, "--Fail: Only found " + ids.rows() + " markers");
        }

        final Mat AR_Center = findCenterRect(markersCenter[0], markersCenter[1], markersCenter[2], markersCenter[3]);
        Log.i(TAG, "distorted=" + AR_Center.dump());

        end = System.currentTimeMillis();
        Log.i(TAG, "ar_read+process_time=" + (end-start));
        return  estimateMarkerPos(corners, ids);
    }

    private Point estimateMarkerPos(List<Mat> markerCorners, Mat ids) {
        final String TAG = "estimateMarkerPos";
        Log.i(TAG, "Start");
        final long start = System.currentTimeMillis();

        Mat rvec = new Mat();
        Mat tvec = new Mat();

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        Aruco.estimatePoseSingleMarkers(markerCorners, MARKER_LENGTH, cameraMat, distCoeffs, rvec, tvec);

        long end = System.currentTimeMillis();

        // Log
        Log.i(TAG, "rvec: " + rvec.dump());
        Log.i(TAG, "rvec size: " + rvec.rows() + " rows" + rvec.cols() + " cols");
        Log.i(TAG, "tvec: " + tvec.dump());
        Log.i(TAG, "tvec: " + tvec.rows() +  " rows" + tvec.cols() + " cols");
        Log.i(TAG, "ids= " + ids.dump());
        Log.i(TAG, "estimate 4 marker position time: " + (end-start));
        return getArCenterTvec(tvec);
    }

    private Point getArCenterTvec(Mat cornerTvec) {
        final String TAG = "getArCenterTvec";
        double xCenter = (cornerTvec.get(0, 0)[0] + cornerTvec.get(1, 0)[0] + cornerTvec.get(2, 0)[0] +
                    cornerTvec.get(3, 0)[0]) / 4.0;
        double yCenter = (cornerTvec.get(0, 0)[1] + cornerTvec.get(1, 0)[1] + cornerTvec.get(2, 0)[1] +
                    cornerTvec.get(3, 0)[1]) / 4.0;
        double zCenter = (cornerTvec.get(0, 0)[2] + cornerTvec.get(1, 0)[2] + cornerTvec.get(2, 0)[2] +
                    cornerTvec.get(3, 0)[2]) / 4.0;

        Log.i(TAG, "AR center tvec in cam's coord system: [" + xCenter + ", " + yCenter + ", " + zCenter);

        // Shift origin
        xCenter = xCenter - (0 + 0.0422) - 0.0572; // y axis in Astrobee's world;
        yCenter = yCenter - (- (0 + 0.08266) - (-0.1111)); // z axis in Astrobee's world
        zCenter = zCenter - (0 - 0.1177); // x axis in Astrobee's world

        Log.i(TAG, "AR center tvec in our coord system(Astrobee's Axis): [" + zCenter + ", " + xCenter + ", " + yCenter);
        Log.i(TAG, "AR center tvec in our coord system(Kibo's Axis): [" + xCenter + ", " + zCenter + ", " + yCenter);

//        -0.1245617163499451, -0.053375311650620275, 0.7558894579555107 - cam's world, cam's axis
//        -0.16676 -0.136035, 0.8735894 - Astrobee's world, came axis
//        -0.16676, 0.8735894, -0.136035 - Astrobee's world, kibo axis
//        0.8735894, -0.16676, -0.136035-  Astrobee's world, Astrobee axis  <---
        return new Point(zCenter, xCenter, yCenter);
    }

    private double[] getLaserAngleToTarget(Point AR_vec) {
        String TAG = "getLaserAngleToTarget";
        final Point laserPoint = new Point(AR_vec.getX(), 0.0f, 0.0f);

        double AR_vec_length_z = Math.sqrt(AR_vec.getX() * AR_vec.getX() + AR_vec.getY() * AR_vec.getY());
        double zAngle = Math.acos(laserPoint.getX() / AR_vec_length_z);

        double AR_vec_length_y = Math.sqrt(AR_vec.getX() * AR_vec.getX() + AR_vec.getZ() * AR_vec.getZ());
        double yAngle = Math.acos(laserPoint.getX() / AR_vec_length_y);

        Log.i(TAG, "AR vec length - z: " + AR_vec_length_z);
        Log.i(TAG, "AR vec length - y: " + AR_vec_length_y);

        zAngle = Math.toDegrees(zAngle);
        yAngle = Math.toDegrees(yAngle);
        if (AR_vec.getY() < 0) zAngle *= (-1);
        if (AR_vec.getZ() > 0) yAngle *= (-1);

        Log.i(TAG, "zAngle: " + zAngle);
        Log.i(TAG, "yAngle: " + yAngle);

        double res[] = { yAngle, zAngle};
        return res;
    }

    private Mat undistortPoints(Mat points) {
        final String TAG = "undistortCorner";

        // in -> rows:1, cols:4
        // in -> 1xN 2 Channel
        Log.i(TAG, "Start");

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        Mat out = new Mat(points.rows(), points.cols(), points.type());

        Imgproc.undistortPoints(points, out, cameraMat, distCoeffs, new Mat(), cameraMat);

        Log.i(TAG, "undistort=" + out.dump());
        // out -> 1xN 2 Channel
        return out;
    }

    private Mat findCenterRect(imagePoint p1, imagePoint p2,
                               imagePoint p3, imagePoint p4) {
        float xCenter = (p1.x + p2.x + p3.x + p4.x) / 4.0f;
        float yCenter = (p1.y + p2.y + p3.y + p4.y) / 4.0f;

        Mat out = new Mat(1, 1, CvType.CV_32FC2);
        float[] point = {xCenter, yCenter};
        out.put(0, 0, point);

        return out;
    }

    private imagePoint findCenterRect(Mat corners) {
        double xCenter;
        double yCenter;

        xCenter = (corners.get(0, 0)[0] + corners.get(0, 1)[0] +
                corners.get(0, 2)[0] + corners.get(0, 3)[0]) / 4.0f;

        yCenter = (corners.get(0, 0)[1] + corners.get(0, 1)[1] +
                corners.get(0, 2)[1] + corners.get(0, 3)[1]) / 4.0f;

        return new imagePoint((float)xCenter, (float)yCenter);
    }

    // OTHER MATHS OPERATION

    private double[] pixelDistanceToAngle(double[] target, double[] ref, int pattern) {
        final String TAG = "pixelDistanceToAngle";

        final double xDistance = target[0] - ref[0];
        final double yDistance = ref[1] - target[1];
        //final double anglePerPixel = 130 / Math.sqrt(Math.pow(NAV_MAX_WIDTH, 2) + Math.pow(NAV_MAX_HEIGHT, 2));
        final double anglePerPixel = 0.08125;
        Log.i(TAG, "xDistance=" + xDistance);
        Log.i(TAG, "yDistance=" + yDistance);
        Log.i(TAG, "anglePerPixel=" + anglePerPixel);

        double xAngle = xDistance * anglePerPixel;
        double yAngle = yDistance * anglePerPixel;

        if (pattern == 7) {
            xAngle -= 1.65;
        }

        if (pattern == 1 || pattern == 8) {
            xAngle -= 1.625;
            yAngle -= 1.34;
        }

        if (pattern == 2 || pattern == 3 || pattern == 4) {
            yAngle -= 1.35;
        }

        Log.i(TAG, "xAngle=" + xAngle);
        Log.i(TAG, "yAngle=" + yAngle);

        double[] out = {xAngle, yAngle};
        return out;
    }

    // Quaternion from v to u
    private Quaternion quaternionFrom2Points(Point u, Point v) {
        final String TAG = "quaternionFrom2Points";
        // Crossproduct u x v
        double cx = u.getY() * v.getZ() - u.getZ() * v.getY();
        double cy = u.getZ() * v.getX() - u.getX() * v.getZ();
        double cz = u.getX() * v.getY() - u.getY() * v.getX();

        double uLength = Math.sqrt(u.getX() * u.getX() + u.getY() * u.getY() + u.getZ() * u.getZ());
        double vLength = Math.sqrt(v.getX() * v.getX() + v.getY() * v.getY() + v.getZ() * v.getZ());

        // Dot Product u . v
        double dotProduct = u.getX() * v.getX() + u.getY() * v.getY() + u.getZ() * v.getZ();

        double w = Math.sqrt((uLength * uLength) * (vLength * vLength)) + dotProduct;

        Log.i(TAG, " x:" + cx + " y:" + cy + " z:" + cz + " w:" + w);
        return new Quaternion((float)cx, (float)cy, (float)cz, (float) w);
    }

    private Quaternion eulerAngleToQuaternion(double xAngle, double yAngle, double zAngle) {
        final String TAG = "Convert euler angle to quaternion";

        xAngle = Math.toRadians(xAngle);
        yAngle = Math.toRadians(yAngle);
        zAngle = Math.toRadians(zAngle);
        double c1 = Math.cos(yAngle/2);
        double c2 = Math.cos(zAngle/2);
        double c3 = Math.cos(xAngle/2);
        double s1 = Math.sin(yAngle/2);
        double s2 = Math.sin(zAngle/2);
        double s3 = Math.sin(xAngle/2);

        double w = c1*c2*c3 - s1*s2*s3;
        double x = s1*s2*c3 + c1*c2*s3;
        double y = s1*c2*c3 + c1*s2*s3;
        double z = c1*s2*c3 - s1*c2*s3;

        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float) y, (float)z, (float)w);
    }

    // For multiply quaternion, Apply q2(new) to q1(old)
    // q1 * q2= a*e - b*f - c*g- d*h + i (b*e + a*f + c*h - d*g) + j (a*g - b*h + c*e + d*f) + k (a*h + b*g - c*f + d*e)
    private Quaternion combineQuaternion(Quaternion newOrientation, Quaternion oldOrientation) {
        String TAG = "combineQuaternion";
        double x =  newOrientation.getX() * oldOrientation.getW() + newOrientation.getY() * oldOrientation.getZ()
                - newOrientation.getZ() * oldOrientation.getY() + newOrientation.getW() * oldOrientation.getX();
        double y = -newOrientation.getX() * oldOrientation.getZ() + newOrientation.getY() * oldOrientation.getW()
                + newOrientation.getZ() * oldOrientation.getX() + newOrientation.getW() * oldOrientation.getY();
        double z =  newOrientation.getX() * oldOrientation.getY() - newOrientation.getY() * oldOrientation.getX()
                + newOrientation.getZ() * oldOrientation.getW() + newOrientation.getW() * oldOrientation.getZ();
        double w = -newOrientation.getX() * oldOrientation.getX() - newOrientation.getY() * oldOrientation.getY()
                - newOrientation.getZ() * oldOrientation.getZ() + newOrientation.getW() * oldOrientation.getW();
        Log.i(TAG, " x:" + x + " y:" + y + " z:" + z + " w:" + w);
        return new Quaternion((float)x, (float)y, (float)z, (float)w);
    }

    // KOZ AVOID BOI

    final float A_PrimeToTarget = 0.08f; // worst case?
    final float KIZ_edgeR = 11.55f;
    final float KIZ_edgeL = 10.3f;
    final float KIZ_edgeB = 5.57f;

    private void move_pattern7(float A_PrimeX, float A_PrimeZ) {
        final String TAG = "move_pattern7";
        long start = System.currentTimeMillis();

        final Quaternion q = new Quaternion(0,0,-0.707f,0.707f);
        final float[] target = {A_PrimeX - A_PrimeToTarget, A_PrimeZ - A_PrimeToTarget};
        final float KOZ_edgeT = target[1] - 0.3f;

        final float x = KIZ_edgeR - 0.005f;

        Log.i(TAG, "KOZ_edgeT=" + KOZ_edgeT);

        Log.i(TAG, "target_x=" + target[0]);
        Log.i(TAG, "target_z=" + target[1]);

        Log.i(TAG, "x=" + x);

        move_to(x, -9.8000, KOZ_edgeT, q);
        move_to(x, -9.8000, A_PrimeZ, q);
        move_to(A_PrimeX, -9.8000, A_PrimeZ, q);

        long end = System.currentTimeMillis();
        Log.i(TAG, "ElapsedTime=" + (end - start));
    }

    private void move_pattern56(float A_PrimeX, float A_PrimeZ) {
        final String TAG = "move_pattern56";
        long start = System.currentTimeMillis();

        Quaternion q = new Quaternion(0,0,-0.707f,0.707f);

        final float[] target = {A_PrimeX + A_PrimeToTarget, A_PrimeZ - A_PrimeToTarget};

        final float KOZ_edgeL = target[0] - 0.3f;
        final float KOZ_edgeB = target[1] + 0.3f;
        final float KOZ_edgeT = target[1] - 0.3f;

        float x = KOZ_edgeL - 0.16f;
        float z = KOZ_edgeB - 0.15f;

        if (x < KIZ_edgeL) {
            x = KIZ_edgeL + 0.16f;
        }

        if (z > KIZ_edgeB) {
            z = KIZ_edgeB - 0.16f;
        }

        Log.i(TAG, "KOZ_edgeL=" + KOZ_edgeL);
        Log.i(TAG, "KOZ_edgeT=" + KOZ_edgeT);
        Log.i(TAG, "KOZ_edgeB=" + KOZ_edgeB);

        Log.i(TAG, "target_x=" + target[0]);
        Log.i(TAG, "target_z=" + target[1]);

        Log.i(TAG, "x=" + x);
        Log.i(TAG, "z=" + z);

        move_to(x, -9.8000, KOZ_edgeT, q);
        move_to(x, -9.8000, A_PrimeZ, q);
        move_to(A_PrimeX, -9.8000, A_PrimeZ, q);

        long end = System.currentTimeMillis();
        Log.i(TAG, "ElapsedTime=" + (end - start));
    }

    private void move_toB(int pattern, float A_PrimeX, float A_PrimeZ) {
        final String TAG = "move_toB";
        long start = System.currentTimeMillis();
        final Quaternion q = new Quaternion(0,0,-0.707f,0.707f);

        final Point PointB = new Point(10.6000, -8.0000, 4.5000);

        if (pattern == 2 || pattern == 3 || pattern == 4) {
            move_to(10.6000, -8.6500, PointB.getZ(), q);
            move_to(PointB, q);
        }

        if (pattern == 1|| pattern == 8) {
            final float[] target = {A_PrimeX - A_PrimeToTarget, A_PrimeZ + A_PrimeToTarget};
            final float KOZ_edgeT = target[1] - 0.3f;
            float KOZ_edgeR = target[0] - 0.075f;

            if (pattern == 8) {
                move_to(A_PrimeX, -9.8000, KOZ_edgeT - 0.32, q);
            }
            else {
                move_to(KOZ_edgeR - 0.16, -9.4000, KOZ_edgeT - 0.16, q);
            }

            move_to(10.6000 , -8.6500, PointB.getZ(), q);
            move_to(PointB, q);
        }

        if (pattern == 5 || pattern == 6) {
            move_to(10.6000, -8.6500, A_PrimeZ, q);
            move_to(PointB, q);
        }

        if (pattern == 7) {
            final float x = KIZ_edgeR - 0.01f;

            move_to(x, -9.8000, A_PrimeZ, q);
            move_to(x, -9.0000, PointB.getZ(), q);
            move_to(10.6000, -9.0000, PointB.getZ(), q);
            move_to(PointB, q);
        }

        long end = System.currentTimeMillis();
        Log.i(TAG, "ElapsedTime=" + (end - start));
    }

}