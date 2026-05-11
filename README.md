ESP32 LIDAR Autopilot
Acest proiect transformă un robot echipat cu ESP32 într-un vehicul capabil de navigare autonomă și control de la distanță, utilizând un senzor LIDAR (YDLIDAR) și un giroscop MPU6050.

 Funcționalități
Harta în timp real: Interfață web (Canvas HTML5) care afișează datele scanate de LIDAR via WebSockets.

Navigare Autonomă: Algoritm de ocolire a obstacolelor bazat pe sectoare de scanare (Frontal, Stânga, Dreapta).

Control Dual: Mod manual (FWD/BCK/LFT/RGT) și mod "Go-To-Angle" prin click pe hartă.

Stabilizare Gyro: Menținerea direcției și calculul unghiului precis folosind MPU6050.

 Arhitectură Sistem
LIDAR (Serial1): Scanează mediul și trimite pachete de distanță la 360°.

Logic de Navigare:

TURNING: Se rotește până atinge unghiul țintă.

DRIVING: Merge înainte dacă drumul este liber.

AVOIDING: Dacă detectează un obstacol (<30cm), decide automat ocolirea prin stânga sau dreapta în funcție de spațiul disponibil.

Web Server: Găzduiește interfața de control și transmite datele binare ale scanării către browser.

 Configurare Hardware
Controller: ESP32

LIDAR: YDLIDAR (conectat pe pinii RX2/TX2 - 16/17)

IMU: MPU6050 (I2C - 21/22)

Driver Motoare: TB6612FNG sau similar (pinii 18, 19, 5, 4, PWM 23, 2)

 Utilizare Rapidă
Introdu datele WiFi (ssid și password) în cod.

Încarcă codul pe ESP32.

Accesează adresa IP a robotului în browser.

Apasă pe hartă pentru a trimite robotul într-o direcție specifică sau folosește butoanele pentru control manual.

Notă: Asigură-te că senzorul MPU6050 este calibrat (robotul trebuie să stea nemișcat la pornire).
