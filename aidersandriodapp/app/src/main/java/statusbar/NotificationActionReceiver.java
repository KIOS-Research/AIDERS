package statusbar;

import android.app.NotificationManager;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

public class NotificationActionReceiver extends BroadcastReceiver {

    public static boolean iscancel=false;

    @Override
    public void onReceive(Context context, Intent intent) {
        Log.d("NotificationAction","register receiver!");

      if (intent.getAction().equalsIgnoreCase("CANCEL")) {

            NotificationManager notificationManager =
                    (NotificationManager) context.getSystemService(Context.NOTIFICATION_SERVICE);
            notificationManager.cancel(1);
          iscancel=true;

        }
    }
}