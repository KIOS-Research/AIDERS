package com.remote.log;

import android.app.Activity;
import android.util.Log;
import android.widget.Toast;

import androidx.annotation.NonNull;

import com.google.android.gms.tasks.OnCompleteListener;
import com.google.android.gms.tasks.Task;
import com.google.firebase.auth.AuthResult;
import com.google.firebase.auth.FirebaseAuth;
import com.google.firebase.auth.FirebaseUser;
import com.google.firebase.database.FirebaseDatabase;

import org.jetbrains.annotations.NotNull;

import java.util.HashMap;

public class FirebaseAuthentication {
    private FirebaseAuth mAuth;
    Activity activity;

    public FirebaseAuthentication(Activity activity) {
        this.activity = activity;
        mAuth = FirebaseAuth.getInstance();

        if (mAuth.getCurrentUser() == null)
            signInUser();
        else {
            FirebaseDatabase database = FirebaseDatabase.getInstance();
            database.goOnline();

            HashMap<String,String> values=new HashMap();
            values.put("uid",mAuth.getUid());
            database.getReference().setValue(values).addOnCompleteListener(new OnCompleteListener<Void>() {
                @Override
                public void onComplete(@NonNull @NotNull Task<Void> task) {
                  //  MainActivity.remoteLogger.log(1, "testing database 3");

                    Log.d(getClass().getSimpleName(), "signInUser COMPLETE LISTENER: " +task.isSuccessful());

                }
            });
            // mAuth.signOut();
            Log.d(getClass().getSimpleName(), "signInUser credentials: " + mAuth.getCurrentUser().getEmail());

        }
    }


    private void createUser() {
        // Check if user is signed in (non-null) and update UI accordingly.
        FirebaseUser currentUser = mAuth.getCurrentUser();
        if (currentUser != null) {
            //  reload();
        } else {

        }


        mAuth.createUserWithEmailAndPassword("test@gmail.com", "djiswarms")
                .addOnCompleteListener(activity, new OnCompleteListener<AuthResult>() {
                    @Override
                    public void onComplete(@NonNull Task<AuthResult> task) {
                        if (task.isSuccessful()) {
                            // Sign in success, update UI with the signed-in user's information
                            Log.d(getClass().getSimpleName(), "createUserWithEmail:success");
                            FirebaseUser user = mAuth.getCurrentUser();
                            //   updateUI(user);
                        } else {
                            // If sign in fails, display a message to the user.
                            Log.w(getClass().getSimpleName(), "createUserWithEmail:failure", task.getException());
                            Toast.makeText(activity, "Authentication failed.",
                                    Toast.LENGTH_SHORT).show();
                            // updateUI(null);
                        }
                    }
                });
    }

    private void signInUser() {
        Log.d(getClass().getSimpleName(), "signInUser: trying");

        Task<AuthResult> authenticationResultTask = mAuth.signInWithEmailAndPassword("ucy.kios@gmail.com", "djiswarms")

                .addOnCompleteListener(activity, new OnCompleteListener<AuthResult>() {
                    @Override
                    public void onComplete(@NonNull Task<AuthResult> task) {
                        if (task.isSuccessful()) {
                            // Sign in success, update UI with the signed-in user's information
                            Log.d(getClass().getSimpleName(), "signInWithEmail:success");
                            FirebaseUser user = mAuth.getCurrentUser();
                            //  updateUI(user);
                        } else {
                            // If sign in fails, display a message to the user.
                            Log.w(getClass().getSimpleName(), "signInWithEmail:failure", task.getException());
                            Toast.makeText(activity, "Authentication failed.",
                                    Toast.LENGTH_SHORT).show();
                            //  updateUI(null);
                        }
                    }
                });
        //  Log.d(getClass().getSimpleName(), "signInUser: result: "+ authenticationResultTask.getResult()!=null?"success":"failed");


    }
}
