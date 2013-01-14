package com.hiveground.ros3dward;

import java.util.ArrayList;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.AsyncTask;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.ListView;
import android.widget.TextView;

public class MainActivity extends Activity {
	private SensorManager mSensorManager;
	private GraphView mGraphView;
	private ListView mList;
	private ArrayList<String> arrayList;
	private MyCustomAdapter mAdapter;
	private TCPClient mTcpClient;

	private class GraphView extends View implements SensorEventListener {
		private Bitmap mBitmap;
		private Paint mPaint = new Paint();
		private Canvas mCanvas = new Canvas();
		private Path mPath = new Path();
		private RectF mRect = new RectF();
		private float mLastValues[] = new float[3 * 2];
		private float mOrientationValues[] = new float[3];
		private int mColors[] = new int[3 * 2];
		private float mLastX;
		private float mScale[] = new float[2];
		private float mYOffset;
		private float mMaxX;
		private float mSpeed = 1.0f;
		private float mWidth;
		private float mHeight;

		public GraphView(Context context) {
			super(context);
			mColors[0] = Color.argb(192, 255, 64, 64);
			mColors[1] = Color.argb(192, 64, 128, 64);
			mColors[2] = Color.argb(192, 64, 64, 255);
			mColors[3] = Color.argb(192, 64, 255, 255);
			mColors[4] = Color.argb(192, 128, 64, 128);
			mColors[5] = Color.argb(192, 255, 255, 64);

			mPaint.setFlags(Paint.ANTI_ALIAS_FLAG);
			mRect.set(-0.5f, -0.5f, 0.5f, 0.5f);
			mPath.arcTo(mRect, 0, 180);
		}

		@Override
		protected void onSizeChanged(int w, int h, int oldw, int oldh) {
			mBitmap = Bitmap.createBitmap(w, h, Bitmap.Config.RGB_565);
			mCanvas.setBitmap(mBitmap);
			mCanvas.drawColor(0xFFFFFFFF);
			mYOffset = h * 0.5f;
			mScale[0] = -(h * 0.5f * (1.0f / (SensorManager.STANDARD_GRAVITY * 2)));
			mScale[1] = -(h * 0.5f * (1.0f / (SensorManager.MAGNETIC_FIELD_EARTH_MAX)));
			mWidth = w;
			mHeight = h;
			if (mWidth < mHeight) {
				mMaxX = w;
			} else {
				mMaxX = w - 50;
			}
			mLastX = mMaxX;
			super.onSizeChanged(w, h, oldw, oldh);
		}

		@Override
		protected void onDraw(Canvas canvas) {
			synchronized (this) {
				if (mBitmap != null) {
					final Paint paint = mPaint;
					final Path path = mPath;
					final int outer = 0xFFC0C0C0;
					final int inner = 0xFFff7010;

					if (mLastX >= mMaxX) {
						mLastX = 0;
						final Canvas cavas = mCanvas;
						final float yoffset = mYOffset;
						final float maxx = mMaxX;
						final float oneG = SensorManager.STANDARD_GRAVITY
								* mScale[0];
						paint.setColor(0xFFAAAAAA);
						cavas.drawColor(0xFFFFFFFF);
						cavas.drawLine(0, yoffset, maxx, yoffset, paint);
						cavas.drawLine(0, yoffset + oneG, maxx, yoffset + oneG,
								paint);
						cavas.drawLine(0, yoffset - oneG, maxx, yoffset - oneG,
								paint);
					}
					canvas.drawBitmap(mBitmap, 0, 0, null);

					float[] values = mOrientationValues;
					if (mWidth < mHeight) {
						float w0 = mWidth * 0.333333f;
						float w = w0 - 32;
						float x = w0 * 0.5f;
						for (int i = 0; i < 3; i++) {
							canvas.save(Canvas.MATRIX_SAVE_FLAG);
							canvas.translate(x, w * 0.5f + 4.0f);
							canvas.save(Canvas.MATRIX_SAVE_FLAG);
							paint.setColor(outer);
							canvas.scale(w, w);
							canvas.drawOval(mRect, paint);
							canvas.restore();
							canvas.scale(w - 5, w - 5);
							paint.setColor(inner);
							canvas.rotate(-values[i]);
							canvas.drawPath(path, paint);
							canvas.restore();
							x += w0;
						}
					} else {
						float h0 = mHeight * 0.333333f;
						float h = h0 - 32;
						float y = h0 * 0.5f;
						for (int i = 0; i < 3; i++) {
							canvas.save(Canvas.MATRIX_SAVE_FLAG);
							canvas.translate(mWidth - (h * 0.5f + 4.0f), y);
							canvas.save(Canvas.MATRIX_SAVE_FLAG);
							paint.setColor(outer);
							canvas.scale(h, h);
							canvas.drawOval(mRect, paint);
							canvas.restore();
							canvas.scale(h - 5, h - 5);
							paint.setColor(inner);
							canvas.rotate(-values[i]);
							canvas.drawPath(path, paint);
							canvas.restore();
							y += h0;
						}
					}

				}
			}
		}

		public void onSensorChanged(SensorEvent event) {
			// Log.d(TAG, "sensor: " + sensor + ", x: " + values[0] + ", y: " +
			// values[1] + ", z: " + values[2]);
			synchronized (this) {
				if (mBitmap != null) {
					final Canvas canvas = mCanvas;
					final Paint paint = mPaint;
					if (event.sensor.getType() == Sensor.TYPE_ORIENTATION) {
						for (int i = 0; i < 3; i++) {
							mOrientationValues[i] = event.values[i];
						}
					} else {
						float deltaX = mSpeed;
						float newX = mLastX + deltaX;

						int j = (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) ? 1
								: 0;
						for (int i = 0; i < 3; i++) {
							int k = i + j * 3;
							final float v = mYOffset + event.values[i]
									* mScale[j];
							paint.setColor(mColors[k]);
							canvas.drawLine(mLastX, mLastValues[k], newX, v,
									paint);
							mLastValues[k] = v;
						}
						if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
							mLastX += mSpeed;
					}
					invalidate();
				}
			}
		}

		public void onAccuracyChanged(Sensor sensor, int accuracy) {
		}
	}

	public class connectTask extends AsyncTask<String, String, TCPClient> {

		@Override
		protected TCPClient doInBackground(String... message) {

			// we create a TCPClient object and
			mTcpClient = new TCPClient(new TCPClient.OnMessageReceived() {
				@Override
				// here the messageReceived method is implemented
				public void messageReceived(String message) {
					// this method calls the onProgressUpdate
					publishProgress(message);
				}
			});
			mTcpClient.run();

			return null;
		}

		@Override
		protected void onProgressUpdate(String... values) {
			super.onProgressUpdate(values);

			// in the arrayList we add the messaged received from server
			arrayList.add(values[0]);
			// notify the adapter that the data set has changed. This means that
			// new message received
			// from server was added to the list
			mAdapter.notifyDataSetChanged();
		}
	}

	/**
	 * Initialization of the Activity after it is first created. Must at least
	 * call {@link android.app.Activity#setContentView setContentView()} to
	 * describe what is to be displayed in the screen.
	 */
	@Override
	protected void onCreate(Bundle savedInstanceState) {
		// Be sure to call the super class.
		super.onCreate(savedInstanceState);

		// mSensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);
		// mGraphView = new GraphView(this);
		// setContentView(mGraphView);

		setContentView(R.layout.activity_main);

		arrayList = new ArrayList<String>();

		final EditText editText = (EditText) findViewById(R.id.editText);
		Button send = (Button) findViewById(R.id.send_button);

		// relate the listView from java to the one created in xml
		mList = (ListView) findViewById(R.id.list);
		mAdapter = new MyCustomAdapter(this, arrayList);
		mList.setAdapter(mAdapter);

		// connect to the server
		new connectTask().execute("");

		send.setOnClickListener(new View.OnClickListener() {
			@Override
			public void onClick(View view) {

				String message = editText.getText().toString();

				// add the text in the arrayList
				arrayList.add("c: " + message);

				// sends the message to the server
				if (mTcpClient != null) {
					mTcpClient.sendMessage(message);
				}

				// refresh the list
				mAdapter.notifyDataSetChanged();
				editText.setText("");
			}
		});

	}

	@Override
	protected void onResume() {
		super.onResume();
		/*
		 * mSensorManager.registerListener(mGraphView,
		 * mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
		 * SensorManager.SENSOR_DELAY_FASTEST);
		 * mSensorManager.registerListener(mGraphView,
		 * mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
		 * SensorManager.SENSOR_DELAY_FASTEST);
		 * mSensorManager.registerListener(mGraphView,
		 * mSensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION),
		 * SensorManager.SENSOR_DELAY_FASTEST);
		 */

		// String ip = sett
		// SharedPreferences sharedPref =
		// PreferenceManager.getDefaultSharedPreferences(this);
		// String ip = sharedPref.getString(SettingsActivity.KEY_PREF_IP, "");
		// String port = sharedPref.getString(SettingsActivity.KEY_PREF_PORT,
		// "");
		// TextView textView = (TextView) findViewById(R.id.textViewServer);
		// textView.setText("Server: " + ip + ":" + port);
	}

	@Override
	protected void onStop() {
		// mSensorManager.unregisterListener(mGraphView);
		super.onStop();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		// Inflate the menu; this adds items to the action bar if it is present.
		getMenuInflater().inflate(R.menu.activity_main, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		// Handle item selection
		switch (item.getItemId()) {
		case R.id.menu_settings: {
			Log.i("info", "hello");
			Intent setting = new Intent(this, SettingsActivity.class);
			// Make it a subactivity so we know when it returns
			// startActivityForResult(setting, REQUEST_CODE_PREFERENCES);
			startActivity(setting);
			return true;
		}
		default:
			return super.onOptionsItemSelected(item);
		}
	}

	/** Called when the user clicks the Send button */
	/*
	 * public void connectServer(View view) { // Do something in response to
	 * button SharedPreferences sharedPref =
	 * PreferenceManager.getDefaultSharedPreferences(this); String ip =
	 * sharedPref.getString(SettingsActivity.KEY_PREF_IP, ""); String port =
	 * sharedPref.getString(SettingsActivity.KEY_PREF_PORT, ""); Log.i("info",
	 * "connect to: " + ip + ":" + port); new connectTask().execute(""); }
	 */
}
