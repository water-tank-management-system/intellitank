importScripts('https://www.gstatic.com/firebasejs/7.14.6/firebase-app.js');
importScripts('https://www.gstatic.com/firebasejs/7.14.6/firebase-messaging.js');

var firebaseConfig = {
    apiKey: 'AIzaSyDzNvvAtMkO320XdM1l3-n4BjTkvyI_Drs',
        authDomain: 'intellitank-a9.firebaseapp.com',
        databaseURL:
          'https://intellitank-a9-default-rtdb.asia-southeast1.firebasedatabase.app',
        projectId: 'intellitank-a9',
        storageBucket: 'intellitank-a9.appspot.com',
        messagingSenderId: '377162448828',
        appId: '1:377162448828:web:a9e33c7aec90a13163b5b9',
};

firebase.initializeApp(firebaseConfig);
const messaging=firebase.messaging();

messaging.getToken(messaging, { vapidKey: '<YOUR_PUBLIC_VAPID_KEY_HERE>' }).then((currentToken) => {
    if (currentToken) {
        console.log('CurrentToken: ', currentToken);
      // Send the token to your server and update the UI if necessary
      // ...
    } else {
      // Show permission request UI
      console.log('No registration token available. Request permission to generate one.');
      // ...
    }
  }).catch((err) => {
    console.log('An error occurred while retrieving token. ', err);
    // ...
  });

messaging.setBackgroundMessageHandler(function (payload) {
    console.log(payload);
    const notification=JSON.parse(payload);
    const notificationOption={
        body:notification.body,
        icon:notification.icon
    };
    return self.registration.showNotification(payload.notification.title,notificationOption);
});

// Fungsi untuk menangani notifikasi dari client side
self.addEventListener('push', function (event) {
    const payload = event.data.json();
    const options = {
        body: payload.notification.body,
        icon: payload.notification.icon,
    };
    event.waitUntil(
        self.registration.showNotification(payload.notification.title, options)
    );
});