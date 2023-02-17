package com.remote.log

import android.util.Log
import com.google.firebase.database.ktx.database
import com.google.firebase.ktx.Firebase
import timber.log.Timber
import java.text.SimpleDateFormat
import java.util.*

class TimberRemoteTree(private val deviceDetails: DeviceDetails) : Timber.DebugTree() {

    private val dateFormat = SimpleDateFormat("dd-MM-yyyy", Locale.getDefault())
    private val timeFormat = SimpleDateFormat("yyyy-MM-dd hh:mm:ss.SSS a zzz", Locale.getDefault())
    private val date = dateFormat.format(Date(System.currentTimeMillis()))

    private val logRef = Firebase.database.getReference("logs/$date/${deviceDetails.deviceId}")

    override fun log(priority: Int, tag: String?, message: String, t: Throwable?) {
        Log.d("TimberRemoteTree","log remote")
      //  if (BuildConfig.DEBUG) {
            Log.d("TimberRemoteTree","debug log remote")

            val timestamp = System.currentTimeMillis()
            val time = timeFormat.format(Date(timestamp))

            val remoteLog = RemoteLog(priorityAsString(priority), tag, message, t.toString(), time)

            with(logRef) {1
                updateChildren(mapOf(Pair("-DeviceDetails", deviceDetails)))
                child(timestamp.toString()).setValue(remoteLog)
            }
     //   } else super.log(priority, tag, message, t)
    }

    private fun priorityAsString(priority: Int): String = when (priority) {
        Log.VERBOSE -> "VERBOSE"
        Log.DEBUG -> "DEBUG"
        Log.INFO -> "INFO"
        Log.WARN -> "WARN"
        Log.ERROR -> "ERROR"
        Log.ASSERT -> "ASSERT"
        else -> priority.toString()
    }
}