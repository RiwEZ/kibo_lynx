package jp.jaxa.iss.kibo.rpc.defaultapk;

// android
import android.graphics.Bitmap;
import android.util.Log;
// kibo rpc api
import gov.nasa.arc.astrobee.Result;
import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
// astrobee gs
import gov.nasa.arc.astrobee.types.*;
import gov.nasa.arc.astrobee.*;
// zxing
import com.google.zxing.BinaryBitmap;
import com.google.zxing.LuminanceSource;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
// opencv
import org.opencv.android.Utils;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;


public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        api.startMission();

        move_to(11.2100, -9.8000, 4.7900, 0, 0, -0.7070f, 0.7070f);

        qr_move();

        api.takeSnapshot();
        api.reportMissionCompletion();
    }

    @Override
    protected void runPlan2(){
        // write here your plan 2
    }

    @Override
    protected void runPlan3(){
        // write here your plan 3
    }

    final int LOOP_MAX = 3;
    final int NAV_MAX_WIDTH = 1280;
    final int NAV_MAX_HEIGHT = 960;

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

    /*
    public Mat getCamMat(double[] in) {
        Mat out = new Mat(3, 3, CvType.CV_32FC1);
        int k = 0, j = 0;
        for (int i = 0; i < in.length; i++) {
            out.put(k, j, in[i]);

            j++;
            if (j == 2) {
                k++;
                j = 0;
            }
        }
        return out;
    }

    public Mat getDistCoeff(double[] in) {
        Mat out = new Mat(1, 5, CvType.CV_32FC1);
        for (int i = 0; i < in.length; i++) {
            out.put(0, i, in[i]);
        }
        return out;
    }
     */

    public Mat undistort(Mat in) {
        final String TAG = "undistort";

        /* generalize this func, should just declare some const for performance
        double[][] cam_info = api.getNavCamIntrinsics();
        Log.i(TAG, "cam matrix =" + Arrays.toString(cam_info[0]));
        Log.i(TAG, "dist coeff =" + Arrays.toString(cam_info[1]));

        Mat cam_Mat = getCamMat(cam_info[0]);
        Mat dist_coeff = getDistCoeff(cam_info[1]);
         */

        Mat cam_Mat = new Mat(3, 3, CvType.CV_32FC1);
        Mat dist_coeff = new Mat(1, 5, CvType.CV_32FC1);

        final double cam_Mat_sim[] = {
                567.229305, 0.0, 659.077221,
                0.0, 574.192915, 517.007571,
                0.0, 0.0, 1.0
        };

        final double dist_coeff_sim[] = {
                -0.216247, 0.03875, -0.010157, 0.001969, 0.0
        };

        cam_Mat.put(3, 3, cam_Mat_sim);
        dist_coeff.put(1, 5, dist_coeff_sim);

        Mat out = new Mat(NAV_MAX_WIDTH, NAV_MAX_HEIGHT, CvType.CV_8UC1);
        Imgproc.undistort(in, out, cam_Mat, dist_coeff);

        return out;
    }

    public BinaryBitmap getNavImg() {
        final String TAG = "procNavImg";

        // img processing shit
        Log.i(TAG, "Processing img");

        api.flashlightControlFront(0.025f);
        Mat pic = undistort(api.getMatNavCam());
        api.flashlightControlFront(0);

        Bitmap uncrop = Bitmap.createBitmap(pic.width(), pic.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(pic, uncrop);

        Log.i(TAG, pic.width() + "");
        Log.i(TAG, pic.height() + "");

        Bitmap bMap = Bitmap.createBitmap(
                uncrop,
                pic.width()/2,
                pic.width()/2,
                (pic.width()/2) * (2/3),
                (pic.height()/2) * (3/4));

        // maybe need to resize or crop or some shit that idk to make it work.

        int[] intArr = new int[bMap.getWidth() * bMap.getHeight()];
        bMap.getPixels(intArr, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
        LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArr);
        BinaryBitmap out = new BinaryBitmap(new HybridBinarizer(source));

        Log.i(TAG, "Success processing img");
        return out;
    }

    public void qr_move() {
        final String TAG = "qr_move";
        Log.i(TAG, "Start");

        String data = null;
        int counter = 0;
        BinaryBitmap bitmap = null;

        while (data == null && counter < LOOP_MAX) {
            try {
                bitmap = getNavImg();
            }
            catch (Exception e) {
                Log.i(TAG, "Failed processing img");
                e.printStackTrace();
                return ;
            }

            try {
                // qr code reading
                Log.i(TAG, "Reading qr code");
                com.google.zxing.Result result = new QRCodeReader().decode(bitmap);
                data = result.getText();
                Log.i(TAG, "Data = " + data);
            }
            catch (Exception e) {
                Log.i(TAG, "Failed reading qr code");
                e.printStackTrace();
            }
        }

    }
}