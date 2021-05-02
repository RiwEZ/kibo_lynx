package jp.jaxa.iss.kibo.rpc.defaultapk;

// android

import android.graphics.Bitmap;
import android.util.Log;
// kibo rpc api
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// astrobee gs
import gov.nasa.arc.astrobee.types.*;
// zxing
import com.google.zxing.BarcodeFormat;
import com.google.zxing.BinaryBitmap;
import com.google.zxing.DecodeHintType;
import com.google.zxing.LuminanceSource;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
// opencv
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;
import java.util.Map;


public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1() {
        api.startMission();

        move_to(11.2100, -9.8000, 4.7900, 0, 0, -0.7070f, 0.7070f);

        qr_move();

        api.takeSnapshot();
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2() {
        // write here your plan 2
    }

    @Override
    protected void runPlan3() {
        // write here your plan 3
    }

    final int LOOP_MAX = 3;
    final int NAV_MAX_WIDTH = 1280;
    final int NAV_MAX_HEIGHT = 960;
    final double[] CAM_MATSIM = {
            567.229305, 0.0, 659.077221,
            0.0, 574.192915, 517.007571,
            0.0, 0.0, 1.0
    };

    final double[] DIST_COEFFSIM = {
            -0.216247, 0.03875, -0.010157, 0.001969, 0.0
    };


    public void log_kinematics() {
        final String TAG = "log_position";
        Point p = api.getTrustedRobotKinematics().getPosition();
        Quaternion q = api.getTrustedRobotKinematics().getOrientation();
        Log.i(TAG, "Position= " + p.getX() + ", " + p.getY() + ", " + p.getZ());
        Log.i(TAG, "Orientation= " + q.toString());
    }

    public void move_to(double x, double y, double z, float qx, float qy, float qz, float qw) {
        final String TAG = "move_to";
        Point p = new Point(x, y, z);
        Quaternion q = new Quaternion(qx, qy, qz, qw);

        int counter = 0;
        Result result;

        Log.i(TAG, "Start");

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        // info
        log_kinematics();
        Log.i(TAG, "Done");
    }

    public Mat undistort(Mat in) {
        final String TAG = "undistort";
        Log.i(TAG, "Start");

        Mat cam_Mat = new Mat(3, 3, CvType.CV_32FC1);
        Mat dist_coeff = new Mat(1, 5, CvType.CV_32FC1);

        cam_Mat.put(0, 0, CAM_MATSIM);
        dist_coeff.put(0, 0, DIST_COEFFSIM);

        Mat out = new Mat(NAV_MAX_WIDTH, NAV_MAX_HEIGHT, CvType.CV_8UC1);
        Log.i(TAG, "Imgrpoc.undistrot");
        Imgproc.undistort(in, out, cam_Mat, dist_coeff);

        Log.i(TAG, "Done");
        return out;
    }

    public Rect crop() {
        return new Rect(590, 480, 300, 400);
    }

    public BinaryBitmap getNavImg() {
        final String TAG = "getNavImg";

        // img processing shit
        Log.i(TAG, "Processing img");

        Mat pic = new Mat(api.getMatNavCam(), crop());

        Log.i(TAG, "bMap init");
        Bitmap bMap = Bitmap.createBitmap(pic.width(), pic.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(pic, bMap);

        Log.i(TAG, "intArr");
        int[] intArr = new int[bMap.getWidth() * bMap.getHeight()];

        Log.i(TAG, "getPixels");
        bMap.getPixels(intArr, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        Log.i(TAG, "Luminance Source");
        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArr);

        Log.i(TAG, "out");
        BinaryBitmap out = new BinaryBitmap(new HybridBinarizer(source));

        Log.i(TAG, "Success processing img");
        return out;
    }

    public void qr_move() {
        final String TAG = "qr_move";
        Log.i(TAG, "Start");

        String text = null;
        int counter = 0;
        BinaryBitmap bitmap;

        Map<DecodeHintType, Object> hints = new Hashtable<>();
        hints.put(DecodeHintType.TRY_HARDER, "");
        List<BarcodeFormat> qr = new ArrayList<>(); qr.add(BarcodeFormat.QR_CODE);
        hints.put(DecodeHintType.POSSIBLE_FORMATS, qr);
        hints.put(DecodeHintType.CHARACTER_SET, "utf-8");

        while (text == null && counter < LOOP_MAX) {
            try {
                bitmap = getNavImg();
                // qr code reading
                Log.i(TAG, "Reading qr code");
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap, hints);
                text = result.getText();
                api.sendDiscoveredQR(text);

                Log.i(TAG, "Data = " + text);
            } catch (Exception e) {
                Log.i(TAG, "Failed reading qr code");
                e.printStackTrace();
            }

            counter++;
        }

        Log.i(TAG, "Done");
    }
}