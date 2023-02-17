package statusbar;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;

import androidx.core.app.NotificationCompat;

import com.kios.rosDJI.R;


/**Show notification on the staus bar of the android device
 *
 */
public class NotificationHelper {

    NotificationCompat.Builder mBuilder;

    private Context mContext;
    private int NOTIFICATION_ID = 1;
    private Notification mNotification;

    private NotificationManager notificationManager;
    private PendingIntent mContentIntent;
    private CharSequence mContentTitle;



    public NotificationHelper(Context context)
    {
        mContext = context;
    }
    public boolean isNotificationVisible(int id) {
        Intent notificationIntent = new Intent(mContext,mContext.getClass());
        PendingIntent test = PendingIntent.getActivity(mContext, id, notificationIntent, PendingIntent.FLAG_CANCEL_CURRENT);
        return test != null;
    }
    /**
     * Put the notification into the status bar
     */
    public void createNotification() {
        NotificationActionReceiver.iscancel=false;
        Intent intentCancel = new Intent(mContext, NotificationActionReceiver.class);

        intentCancel.setAction("CANCEL");
        intentCancel.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
         notificationManager = (NotificationManager) mContext.getSystemService(Context.NOTIFICATION_SERVICE);

        int notificationId = 1;
        String channelId = "channel-01";
        String channelName = "Channel Name";
        int importance = NotificationManager.IMPORTANCE_HIGH;

        if (android.os.Build.VERSION.SDK_INT >= android.os.Build.VERSION_CODES.O) {
            NotificationChannel mChannel = new NotificationChannel(
                    channelId, channelName, importance);
            notificationManager.createNotificationChannel(mChannel);
        }

        mBuilder = new NotificationCompat.Builder(mContext, channelId)
                .setSmallIcon(R.mipmap.ic_launcher)
                .setContentTitle("Download Media")
                .setContentText("")
        .setVibrate(null)
        .setOnlyAlertOnce(true)

       ;
        mBuilder.setDefaults(0);
        /////cancel button notification
        PendingIntent pendingIntentCancel = PendingIntent.getBroadcast(mContext, 1, intentCancel, PendingIntent.FLAG_CANCEL_CURRENT);
        mBuilder.addAction(R.drawable.selector_camera_control_shoot_hdr_photo,"CANCEL",pendingIntentCancel);
        intentCancel.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);

        //////////

        Intent intent=new Intent(mContext,mContext.getClass());
       // TaskStackBuilder stackBuilder = TaskStackBuilder.create(mContext);
       // stackBuilder.addNextIntent(intent);
        PendingIntent pendingIntent = PendingIntent.getActivity(mContext, 0 /* Request
  code */, intent,
                PendingIntent.FLAG_ONE_SHOT);

        mBuilder.setContentIntent(pendingIntent);
        mNotification=mBuilder.build();
        notificationManager.notify(notificationId, mBuilder.build());
    }

    /**
     * Receives progress updates from the background task and updates the status bar notification appropriately
     * @param percentageComplete
     */
    public void progressUpdate(String text) {
        //build up the new status message
        CharSequence contentText = text;
        //publish it to the status bar
        mBuilder.setContentText(contentText+"");
        notificationManager.notify(NOTIFICATION_ID, mBuilder.build());
    }

    /**
     * called when the background task is complete, this removes the notification from the status bar.
     * We could also use this to add a new ‘task complete’ notification
     */
    public void completed()    {
        if (notificationManager!=null)
        //remove the notification from the status bar
        notificationManager.cancel(NOTIFICATION_ID);
    }
}