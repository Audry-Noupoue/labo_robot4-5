#include <MeAuriga.h> 
#include <MeUltrasonicSensor.h>
MeUltrasonicSensor ultra(PORT_10);  


#define BUZZER_PIN 45
#define LED_RING_PIN 44
#define AURIGARINGLEDNUM 12
#define SPEED_BASE 100


MeEncoderOnBoard encoderLeft(SLOT2);
MeEncoderOnBoard encoderRight(SLOT1);
MeBuzzer buzzer;
MeRGBLed led_ring(0, AURIGARINGLEDNUM);
MeGyro gyro(0, 0x69);

unsigned long currentTime = 0;
bool debugMode = false;


bool wPressed = false;
bool aPressed = false;
bool sPressed = false;
bool dPressed = false;
bool ePressed = false;


bool buzzerOn = false;
unsigned long dernierBip = 0;
unsigned long dernierCligno = 0;
bool etatCligno = false;


int vitesse = SPEED_BASE;


int ledMode = 0; 
int savedR = 0, savedG = 0, savedB = 0;


bool modeAuto = false;
bool ledManualOverride = false;
unsigned long previousBlinkTime = 0;
bool ledState = false;
float targetDistance = 0; 
unsigned long autoStartTime = 0; 


String lastCommand = "";
float distanceRestante = 0.0;


void setup() {
  Serial.begin(115200);

  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  encoderLeft.setPulse(9);
  encoderRight.setPulse(9);

  buzzer.setpin(BUZZER_PIN);
  led_ring.setpin(LED_RING_PIN);

  led_ring.setColor(0, 0, 0);
  led_ring.show();

  Serial.println("Robot prêt !");
}

void loop() {
  currentTime = millis();
  encoderLeft.loop();
  encoderRight.loop();

  if (currentTime - dernierCligno >= 300) {
    dernierCligno = currentTime;
    etatCligno = !etatCligno;
  }

  if (wPressed) avancer();
  else if (sPressed) reculer();
  else if (aPressed) pivoterGauche();
  else if (dPressed) pivoterDroite();
  else arreter();

  if (ePressed) {
    buzzer.tone(1000, 300);
    ePressed = false;
  }

  communicationTask();
  stateManager(currentTime);
   modeDebug(); 

}

void communicationTask() {}

void modeDebug() {
  static unsigned long lastDebugTime = 0;  
  const unsigned long debugInterval = 200; 

  
  if (!debugMode) return;

  unsigned long currentTime = millis();
  if (currentTime - lastDebugTime >= debugInterval) {
    lastDebugTime = currentTime;

    Serial.println(F("----- MODE DEBUG -----"));
    Serial.print(F("Dernière commande reçue : "));
    Serial.println(lastCommand); 
    Serial.print(F("Distance restante : "));
    Serial.print(distanceRestante);
    Serial.println(F(" cm"));
    Serial.println(F("-----------------------"));
  }
}


void serialEvent() {
  static String receivedData = "";
  if (!Serial.available()) return;
  receivedData = Serial.readStringUntil('\n');
  parseData(receivedData);
}

void parseData(String& receivedData) {
  bool isFromBLE = false; 

  if (receivedData.length() >= 2) {
    // Vérifier si les deux premiers octets sont 0xFF55 (BLE)
    if ((uint8_t)receivedData[0] == 0xFF && (uint8_t)receivedData[1] == 0x55) {
      isFromBLE = true;
      // Supprimer les deux premiers octets
      receivedData.remove(0, 2);
    }
    // Vérifier si les deux premiers caractères sont "!!" (Moniteur Série)
    else if (receivedData.startsWith("!!")) {
      // Supprimer les deux premiers caractères
      receivedData.remove(0, 2);
    } else {
      // En-tête non reconnue
      Serial.print(F("Données non reconnues : "));
      Serial.println(receivedData);
      return;
    }
  } else {
    Serial.print(F("Données trop courtes : "));
    Serial.println(receivedData);
    return;
  }

  // Afficher les données reçues si le mode débogage est activé
  if (debugMode) {
    Serial.print(F("Reçu : "));
    Serial.println(receivedData);
    Serial.print(F("Source : "));
    Serial.println(isFromBLE ? F("BLE") : F("Moniteur Série"));
  }

  // Découpage de la commande et des paramètres
  int firstComma = receivedData.indexOf(',');

  if (firstComma == -1) {
    // Pas de virgule, donc c'est une commande sans paramètres
    handleCommand(receivedData);
  } else {
    // Il y a des paramètres
    String command = receivedData.substring(0, firstComma);
    String params = receivedData.substring(firstComma + 1);
    handleCommandWithParams(command, params);
  }
}


void handleCommand(String command) {
  char cmd = command[0];

  switch (cmd) {
    case 'F': wPressed = true; ledMode = 1; break;
    case 'B': sPressed = true; ledMode = 1; break;
    case 'L': aPressed = true; ledMode = 1; break;
    case 'R': dPressed = true; ledMode = 1; break;
    case 'K': ePressed = true; break;
    case 'S':
      wPressed = aPressed = sPressed = dPressed = ePressed = false;
      arreter();
      break;
    case 'd':
      debugMode = !debugMode;
      Serial.print(F("Mode débogage : "));
      Serial.println(debugMode ? F("activé") : F("désactivé"));
      break;
   
    case 'p':
      vitesse = SPEED_BASE;
      Serial.print("Vitesse réglée à : ");
      Serial.println(vitesse);
      break;
    case 'x':
      Serial.println("Commande AUTO reçue !");
      break;
    default:
      Serial.print(F("Commande inconnue : "));
      Serial.println(command);
      break;
  }
}

void handleCommandWithParams(String command, String params) {
  lastCommand = command + (params.length() > 0 ? "," + params : "");
  char cmd = command[0];
  switch (cmd) {
    case 'l':  // Commande lumière
      Serial.print(F("Commande LIGHT reçue : "));
      Serial.println(params);
      commandLight(params);
      break;
    case 'A': // Commande AUTO
      Serial.print(F("Commande AUTO reçue avec paramètres : "));
      Serial.println(params);
      commandAuto(params);
      break;  
    default:
      Serial.print(F("Commande inconnue avec paramètres : "));
      Serial.println(command);
      break;
  }
}

void stateManager(unsigned long ct) {
  if (modeAuto) {
    
    if (ct - previousBlinkTime >= 300) {
      previousBlinkTime = ct;
      ledState = !ledState;
      if (ledState) ledAction(255, 255, 0);  
      else ledAction(0, 0, 0);
    }

    avancer1();

  
    static float startDist = 0;
    static bool init = false;

    if (!init) {
      startDist = ultra.distanceCm();
      init = true;
    }

    float currentDist = ultra.distanceCm();
    float travelled = abs(currentDist - startDist);

    if (travelled >= targetDistance) {
      modeAuto = false;
      init = false;
      arreter();
      ledAction(0, 255, 0); 
      Serial.println(F("Mode AUTO terminé "));
    }
  }
}



void commandAuto(String params) {
  targetDistance = params.toFloat();
  if (targetDistance <= 0) {
    Serial.println(F("Distance invalide pour AUTO."));
    return;
  }

  Serial.print(F("Mode AUTO activé pour distance : "));
  Serial.print(targetDistance);
  Serial.println(F(" cm"));

  modeAuto = true;
  autoStartTime = millis();
  previousBlinkTime = millis();
  ledAction(255, 255, 0); 
}


void avancer() {
  if (modeAuto || ledManualOverride) return;
    short speed = 150;
    short firstRun = 0;
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // Boucle de contrôle PD
    // Modifier les valeurs pour ajuster la réaction du robot
    // kp = coefficient proportionnel
    // kp plus élevé = plus réactif, peut avoir de l'oscillation
    // kp plus bas = moins réactif, mais moins d'oscillation
    //
    // kd = coefficient dérivé
    // kd plus élevé = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 6.5;
    const double kd = 1.1;    
    
    // Premier appel de la fonction
    // On initialise les variables
    if (firstRun) {
      gyro.resetData();
      zAngleGoal = gyro.getAngleZ();
      firstRun = 0;
      Serial.println ("Setting speed");
      
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
       clearLeds();
    }
    
    // On calcule l'erreur
    error = gyro.getAngleZ() - zAngleGoal;
    
    // On calcule la sortie
    output = kp * error + kd * (error - previousError);
    
    // On garde en mémoire l'erreur précédente
    previousError = error;
    
    // On applique la correction
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);

     digitalWrite(BUZZER_PIN, LOW);
    if (ledMode == 0) return;
    clearLeds();
    for (int i = 0; i < 6; i++) led_ring.setColor(i, 0, 255, 0);
    led_ring.show();
}

void avancer1() {
    short speed = 150;
    short firstRun = 0;
    static double zAngleGoal = 0.0;
    
    static double error = 0.0;
    static double previousError = 0.0;
    static double output = 0;
    
    // Boucle de contrôle PD
    // Modifier les valeurs pour ajuster la réaction du robot
    // kp = coefficient proportionnel
    // kp plus élevé = plus réactif, peut avoir de l'oscillation
    // kp plus bas = moins réactif, mais moins d'oscillation
    //
    // kd = coefficient dérivé
    // kd plus élevé = limite l'oscillation, la bonne valeur arrête l'oscillation
    const double kp = 6.5;
    const double kd = 1.1;    
    
    // Premier appel de la fonction
    // On initialise les variables
    if (firstRun) {
      gyro.resetData();
      zAngleGoal = gyro.getAngleZ();
      firstRun = 0;
      Serial.println ("Setting speed");
      
      encoderLeft.setMotorPwm(speed);
      encoderRight.setMotorPwm(-speed);
      
      return;
       clearLeds();
    }
    
    // On calcule l'erreur
    error = gyro.getAngleZ() - zAngleGoal;
    
    // On calcule la sortie
    output = kp * error + kd * (error - previousError);
    
    // On garde en mémoire l'erreur précédente
    previousError = error;
    
    // On applique la correction
    encoderLeft.setMotorPwm(speed - output);
    encoderRight.setMotorPwm(-speed - output);

     digitalWrite(BUZZER_PIN, LOW);
}

void reculer() {
  if (modeAuto || ledManualOverride) return;
  if (ledMode == 0) return;
  if (currentTime - dernierBip >= 500) {
    dernierBip = currentTime;
    buzzerOn = !buzzerOn;
  }
  analogWrite(BUZZER_PIN, buzzerOn ? 127 : 0);
  encoderLeft.setMotorPwm(-vitesse);
  encoderRight.setMotorPwm(vitesse);
  clearLeds();
  if (etatCligno) {
    for (int i = 6; i < 12; i++) led_ring.setColor(i, 255, 0, 0);
  }
  led_ring.show();
}

void pivoterGauche() {
 if (modeAuto || ledManualOverride) return;
  if (ledMode == 0) return;
  encoderLeft.setMotorPwm(-vitesse );
  encoderRight.setMotorPwm(-vitesse );
  clearLeds();
  if (etatCligno) {
    for (int i = 1; i < 4; i++) led_ring.setColor(i, 255, 255, 0);
  }
  led_ring.show();
}

void pivoterDroite() {
  if (modeAuto || ledManualOverride) return;
  if (ledMode == 0) return;
  encoderLeft.setMotorPwm(vitesse );
  encoderRight.setMotorPwm(vitesse );
  clearLeds();
  if (etatCligno) {
    for (int i = 4; i < 7; i++) led_ring.setColor(i, 255, 255, 0);
  }
  led_ring.show();
}

void arreter() {
  if (modeAuto) return;
  

  encoderLeft.setMotorPwm(0);
  encoderRight.setMotorPwm(0);
  digitalWrite(BUZZER_PIN, LOW);

  if (ledMode == 0) {
   
    led_ring.setColor(savedR, savedG, savedB);
    led_ring.show();
    return;
  }

  clearLeds();
  for (int i = 6; i < 12; i++) led_ring.setColor(i, 255, 0, 0);
  led_ring.show();
}

void clearLeds() {
  for (int i = 0; i < 12; i++) led_ring.setColor(i, 0, 0, 0);
}

#pragma region COMMANDES

void ledAction(int r, int g, int b) {
  led_ring.setColor(r, g, b);
  led_ring.show();   
}

void ledAction(int idx, int  r, int g, int b) {
 
 
  if (idx == 0) {
    led_ring.setColor(r, g, b);  
  }
  else {
    led_ring.setColorAt(idx, r, g, b);  
  }
  
  led_ring.show(); 
}


void commandLight(String params) {
   int commaCount = countCharOccurrences(params, ',');
   ledMode = 0;
   ledManualOverride = true; 
  // Vérifie le nombre de paramètres en comptant les virgules
  if (commaCount == 2) {
    // Trois paramètres (r, g, b) pour définir toute la couleur de l'anneau
    int r = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int g = params.substring(0, params.indexOf(',')).toInt();
    int b = params.substring(params.indexOf(',') + 1).toInt();
    
    ledAction(r, g, b);  // Appel pour affecter l'ensemble de l'anneau
    savedR = r; savedG = g; savedB = b;
  } 
  else if (commaCount == 3) {
    // Quatre paramètres (idx, r, g, b) pour définir une LED spécifique
    int idx = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int r = params.substring(0, params.indexOf(',')).toInt();
    params = params.substring(params.indexOf(',') + 1);
    int g = params.substring(0, params.indexOf(',')).toInt();
    int b = params.substring(params.indexOf(',') + 1).toInt();
    
    ledAction(idx, r, g, b);  
    
  } 
  else if (params.startsWith("reset")) {
    ledManualOverride = false;
    clearLeds();
    led_ring.show();
    Serial.println(F("Mode LED automatique restauré."));
  }
  else {
    Serial.println(F("Commande lumière invalide"));
  }
}


#pragma endregion

#pragma region HELPERS
int countCharOccurrences(const String &str, char ch) {
  int count = 0;
  for (int i = 0; i < str.length(); i++) {
    if (str[i] == ch) {
      count++;
    }
  }
  return count;
}
#pragma endregion 