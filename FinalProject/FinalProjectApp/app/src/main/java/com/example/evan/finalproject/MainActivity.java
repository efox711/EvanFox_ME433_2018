package com.example.evan.finalproject;

import android.os.Bundle;
import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.TextureView;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.SeekBar;
import android.widget.SeekBar.OnSeekBarChangeListener;

import java.io.IOException;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;
import static android.graphics.Color.rgb;

public class MainActivity extends Activity implements TextureView.SurfaceTextureListener {
    private Camera mCamera;
    private TextureView mTextureView;
    private SurfaceView mSurfaceView;
    private SurfaceHolder mSurfaceHolder;
    private Bitmap bmp = Bitmap.createBitmap(640, 480, Bitmap.Config.ARGB_8888);
    private Canvas canvas = new Canvas(bmp);
    private Paint paint1 = new Paint();
    private TextView mTextView;
    SeekBar myControl;
    TextView myTextView;

    static long prevtime = 0; // for FPS calculation
    static int thresh = 0;
    static int ptgrn = 0;
    static int ptred = 0;
    static int ptblu = 0;


    private void setMyControlListener() {
        myControl.setOnSeekBarChangeListener(new OnSeekBarChangeListener() {

            int progressChanged = 0;

            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                progressChanged = progress;
                thresh = progress;
                myTextView.setText("The threshold value is: "+progress);

            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {

            }

        });
    }

    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON); // keeps the screen from turning off

        mTextView = (TextView) findViewById(R.id.cameraStatus);

        myControl = (SeekBar) findViewById(R.id.seek1);

        myTextView = (TextView) findViewById(R.id.textView01);
        myTextView.setText("Slider for color threshold");

        setMyControlListener();

        // see if the app has permission to use the camera
        ActivityCompat.requestPermissions(MainActivity.this, new String[]{Manifest.permission.CAMERA}, 1);
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
            mSurfaceView = (SurfaceView) findViewById(R.id.surfaceview);
            mSurfaceHolder = mSurfaceView.getHolder();

            mTextureView = (TextureView) findViewById(R.id.textureview);
            mTextureView.setSurfaceTextureListener(this);

            // set the paintbrush for writing text on the image
            paint1.setColor(0xffff0000); // red
            paint1.setTextSize(24);

            mTextView.setText("started camera");
        } else {
            mTextView.setText("no camera permissions");
        }

    }

    public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
        mCamera = Camera.open();
        Camera.Parameters parameters = mCamera.getParameters();
        parameters.setPreviewSize(640, 480);
        parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_INFINITY); // no autofocusing
        parameters.setAutoExposureLock(true); // keep the white balance constant  <--- Might want to kill, white balance lock is on rn
        mCamera.setParameters(parameters);
        mCamera.setDisplayOrientation(90); // rotate to portrait mode

        try {
            mCamera.setPreviewTexture(surface);
            mCamera.startPreview();
        } catch (IOException ioe) {
            // Something bad happened
        }
    }

    public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
        // Ignored, Camera does all the work for us
    }

    public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
        mCamera.stopPreview();
        mCamera.release();
        return true;
    }

    // the important function
    public void onSurfaceTextureUpdated(SurfaceTexture surface) {
        // every time there is a new Camera preview frame
        mTextureView.getBitmap(bmp);

        final Canvas c = mSurfaceHolder.lockCanvas();
        int sum_mr = 0; // the sum of the mass times the radius
        int sum_m = 0; // the sum of the masses
        int COM;
        if (c != null) {
//            for (int j = 0; j < bmp.getHeight(); j++) {
            for (int j = 200; j < 250; j++) {
                // int thresh = 0; // comparison value
                int[] pixels = new int[bmp.getWidth()]; // pixels[] is the RGBA data
                int startY = j; // which row in the bitmap to analyze to read
                bmp.getPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);

                /*// in the row, see if there is more green than red
                for (int i = 0; i < bmp.getWidth(); i++) {
                    if (((green(pixels[i]) - red(pixels[i])) > 0.5*thresh) && ((green(pixels[i] - blue(pixels[i]))) > 0.5*thresh) && (blue(pixels[i]) < 255-2.55*thresh) && (red(pixels[i]) < 255-2.55*thresh)){ //&& (green(pixels[i]) > 100)) {
                        pixels[i] = rgb(0, 255, 0); // over write the pixel with pure green
                    }
                }*/
                for (int i = 0; i < bmp.getWidth(); i++) {
                    if (((red(pixels[i]) - green(pixels[i])) > 0.5*thresh) && ((red(pixels[i] - blue(pixels[i]))) > 0.5*thresh) && (blue(pixels[i]) < 255-2.55*thresh) && (green(pixels[i]) < 255-2.55*thresh)){
                        pixels[i] = rgb(200, 1, 1); // set the pixel to almost 100% black

                        //sum_m = sum_m + green(pixels[i])+red(pixels[i])+blue(pixels[i]);
                        sum_m = sum_m +red(pixels[i]);
                        sum_mr = sum_mr + (green(pixels[i])+red(pixels[i])+blue(pixels[i]))*i;
                    }
                }


                // update the row
                bmp.setPixels(pixels, 0, bmp.getWidth(), 0, startY, bmp.getWidth(), 1);
            }

        }
        // only use the data if there were a few pixels identified, otherwise you might get a divide by 0 error
        if(sum_m>5){
            COM = sum_mr / sum_m;
        }
        else {
            COM = 0;
        }

        // draw a circle at some position
        int pos = COM;
        canvas.drawCircle(pos, 225, 5, paint1); // x position, y position, diameter, color

        // write the pos as text
        canvas.drawText("pos = " + pos, 10, 200, paint1);
        c.drawBitmap(bmp, 0, 0, null);
        mSurfaceHolder.unlockCanvasAndPost(c);

        // calculate the FPS to see how fast the code is running
        long nowtime = System.currentTimeMillis();
        long diff = nowtime - prevtime;
        mTextView.setText("FPS " + 1000 / diff + "     COM " + COM);
        prevtime = nowtime;
    }
}