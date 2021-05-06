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
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
//
import org.opencv.aruco.Aruco;
import org.opencv.android.Utils;
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

        move_to(11.2100, -9.8000, 4.7900, 0, 0, -0.7070f, 0.7070f);

        String qrContents = qr_read();
        float qrData[] = interpretQRString(qrContents); // {kozPattern, x, y, z}
        Log.i("Interpret QR String", Arrays.toString(qrData));

        int pattern = (int)qrData[0];
        move_to(qrData[1], qrData[2], qrData[3], 0, 0, -0.707f, 0.707f);
        ar_read();

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

    // GLOBAL CONSTANT

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

    // UTILITIES

    private void log_kinematics() {
        final String TAG = "log_position";
        Point p = api.getTrustedRobotKinematics().getPosition();
        Quaternion q = api.getTrustedRobotKinematics().getOrientation();
        Log.i(TAG, "Position= " + p.getX() + ", " + p.getY() + ", " + p.getZ());
        Log.i(TAG, "Orientation= " + q.toString());
    }

    // MOVING

    private void move_to(double x, double y, double z, float qx, float qy, float qz, float qw) {
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

    // QR CODE READING

    private BinaryBitmap getNavImg() {
        final String TAG = "getNavImg";

        // img processing shit
        Log.i(TAG, "Processing img");

        Mat pic = new Mat(api.getMatNavCam(), new Rect(590, 480, 300, 400));

        Bitmap bMap = Bitmap.createBitmap(pic.width(), pic.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(pic, bMap);

        int[] intArr = new int[bMap.getWidth() * bMap.getHeight()];

        bMap.getPixels(intArr, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());

        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArr);

        BinaryBitmap out = new BinaryBitmap(new HybridBinarizer(source));

        return out;
    }

    private String qr_read() {
        final String TAG = "qr_read";
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
                Log.i(TAG, "Done -- Success");

                return text;
            }
            catch (Exception e) {
                Log.i(TAG, "Failed reading qr code");
                e.printStackTrace();
            }

            counter++;
        }

        Log.i(TAG, "Done with failure");
        return null;
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

    // AR CODE READING

    private Mat undistortCorner(Mat in, Mat cameraMat, Mat distCoeffs) {
        final String TAG = "undistortCorner";

        Mat out = new Mat(in.rows(), in.cols(), in.type());

        Log.i(TAG, "in rows:" + in.rows());
        Log.i(TAG, "in cols" + in.cols());

        Imgproc.undistortPoints(in, out, cameraMat, distCoeffs);

        return out;
    }

    private void ar_read() {
        final String TAG = "ar_read";

        Mat pic = api.getMatNavCam();
        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        List<Mat> corners = new ArrayList<>();
        Mat ids = new Mat();

        Mat cameraMat = new Mat(3, 3, CvType.CV_32FC1);
        Mat distCoeffs = new Mat(1, 5, CvType.CV_32FC1);

        cameraMat.put(0, 0, CAM_MATSIM);
        distCoeffs.put(0, 0, DIST_COEFFSIM);

        try {
            Log.i(TAG, "Reading AR tags");
            Aruco.detectMarkers(pic, dict, corners, ids);

            Mat t = undistortCorner(corners.get(0), cameraMat, distCoeffs);
            Log.i(TAG, "corners[" + 0 + "]=" + corners.get(0).dump());
            Log.i(TAG, "undistorted corners[" + 0 + "]=" + t.dump());

            Log.i(TAG, "ids= " + ids.dump());
        }
        catch (Exception e) {
            Log.i(TAG, "Somethings went wrong");
        }
    }
    
    
}