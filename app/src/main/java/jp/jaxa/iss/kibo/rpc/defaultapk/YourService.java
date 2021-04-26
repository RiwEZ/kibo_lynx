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
import org.opencv.core.Mat;


public class YourService extends KiboRpcService {
    @Override
    protected void runPlan1(){
        // write here your plan 1
        api.startMission();
        move_to(11.2100f, -9.8000f, 4.7900f, 0, 0, -0.7070f, 0.7070f);
        qr_move();

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


    public void move_to(float x, float y, float z, float qx, float qy, float qz, float qw) {
        final String TAG = "move_to";
        Point p = new Point(x, y, z);
        Quaternion q = new Quaternion(qx, qy, qz, qw);

        int counter = 0;
        final int LOOP_MAX = 3;
        Result result;

        Log.i(TAG, "Start");

        do {
            result = api.moveTo(p, q, true);
            counter++;
        } while (!result.hasSucceeded() && (counter < LOOP_MAX));

        Log.i(TAG, "Done");
    }

    public void qr_move() {
        String data;

        final String TAG = "qr_move";
        Log.i(TAG, "Start");
        api.flashlightControlFront(0.025f);
        
        BinaryBitmap bitmap = null;
        
        try {
            // img processing shit
            Log.i(TAG, "Processing img");

            Bitmap bMap = api.getBitmapNavCam();

            int[] intArr = new int[bMap.getWidth() * bMap.getHeight()];
            bMap.getPixels(intArr, 0, bMap.getWidth(), 0, 0, bMap.getWidth(), bMap.getHeight());
            LuminanceSource source = new RGBLuminanceSource(bMap.getWidth(), bMap.getHeight(), intArr);
            bitmap = new BinaryBitmap(new HybridBinarizer(source));
            Log.i(TAG, "Success processing img");
        }
        catch (Exception e) {
            Log.i(TAG, "Failed processing img");
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
            return ;
        }
    }
}