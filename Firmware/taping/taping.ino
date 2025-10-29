/*
 * ESP32 Gripper Servo Control Web Interface - PWM Control
 * Controls 6 servos on pins: 2, 14, 15, 17, 19, 25
 * Creates WiFi hotspot for web-based control
 * Uses PWM microsecond control for maximum precision
 * 
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// WiFi credentials for hotspot
const char* ssid = "ESP32_Gripper_Control";
const char* password = "gripper123";

// Web server on port 80
WebServer server(80);

// Servo objects
Servo servo1, servo2, servo3, servo4, servo5, servo6;

// Servo pins - AMI Taping Machine configuration
const int servoPins[] = {2, 25, 15, 14, 19, 17};  // Based on AMI_Taping_v9.ino
Servo servos[] = {servo1, servo2, servo3, servo4, servo5, servo6};

// PWM Control Settings (microseconds)
const int PWM_MIN = 500;    // Minimum pulse width (μs)
const int PWM_MAX = 2500;   // Maximum pulse width (μs)
const int PWM_CENTER = 1500; // Center position (μs)

// Current servo positions in microseconds
int servoPWM[] = {1500, 1500, 1500, 1500, 1500, 1500};

// Servo calibration ranges (can be adjusted per servo)
struct ServoCalibration {
  int minPWM;
  int maxPWM;
  int centerPWM;
  String name;
};

ServoCalibration servoCalib[] = {
  {500, 2500, 1500, "Feeder"},
  {500, 2500, 1500, "Wrapper"}, 
  {500, 2500, 1500, "Cutter"},
  {500, 2500, 1500, "Holder"},
  {500, 2500, 1500, "Gripper"},
  {700, 1550, 1125, "Elevator"}  // Industrial grade: Down=700, Up=1550, Center=1125
};

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Iniciando ESP32 Gripper Control PWM...");
  
  // Configure PWM timers for ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Initialize servos with PWM control
  for (int i = 0; i < 6; i++) {
    servos[i].setPeriodHertz(50);    // Standard 50Hz servo frequency
    servos[i].attach(servoPins[i], servoCalib[i].minPWM, servoCalib[i].maxPWM);
    
    // Set initial position to center
    servos[i].writeMicroseconds(servoCalib[i].centerPWM);
    servoPWM[i] = servoCalib[i].centerPWM;
    
    Serial.print("✓ ");
    Serial.print(servoCalib[i].name);
    Serial.print(" inicializado en pin ");
    Serial.print(servoPins[i]);
    Serial.print(" - PWM: ");
    Serial.print(servoCalib[i].centerPWM);
    Serial.println("μs");
    
    delay(100); // Small delay between servo initializations
  }
  
  // Configure WiFi hotspot
  Serial.println("\nConfigurando hotspot WiFi...");
  WiFi.softAP(ssid, password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("✓ Hotspot creado: ");
  Serial.println(ssid);
  Serial.print("✓ IP del servidor: ");
  Serial.println(IP);
  Serial.print("✓ Contraseña: ");
  Serial.println(password);
  
  // Configure web server routes
  server.on("/", handleRoot);
  server.on("/control", handleServoControl);
  server.on("/status", handleStatus);
  server.on("/calibrate", handleCalibration);
  server.on("/sequence", handleSequence);
  
  // Start web server
  server.begin();
  Serial.println("✓ Servidor web iniciado en puerto 80");
  Serial.println("\n🌐 Accede a la interfaz web:");
  Serial.print("   http://");
  Serial.print(IP);
  Serial.println("/");
  Serial.println("\n🎯 Control PWM de precisión activado!");
  Serial.println("   Rango: 500-2500μs | Centro: 1500μs");
}

void loop() {
  server.handleClient();
  delay(2);
}

// Function to move servo using PWM microseconds
void moveServoPWM(int servoIndex, int pwmValue) {
  if (servoIndex >= 0 && servoIndex < 6) {
    // Constrain PWM value to safe range
    pwmValue = constrain(pwmValue, servoCalib[servoIndex].minPWM, servoCalib[servoIndex].maxPWM);
    
    // Write PWM value directly
    servos[servoIndex].writeMicroseconds(pwmValue);
    servoPWM[servoIndex] = pwmValue;
    
    Serial.print(servoCalib[servoIndex].name);
    Serial.print(" (Pin ");
    Serial.print(servoPins[servoIndex]);
    Serial.print(") PWM: ");
    Serial.print(pwmValue);
    Serial.println("μs");
  }
}

// Convert PWM to percentage (0-100%)
int pwmToPercent(int pwmValue, int servoIndex) {
  int minPWM = servoCalib[servoIndex].minPWM;
  int maxPWM = servoCalib[servoIndex].maxPWM;
  return map(pwmValue, minPWM, maxPWM, 0, 100);
}

// Convert percentage to PWM
int percentToPWM(int percent, int servoIndex) {
  int minPWM = servoCalib[servoIndex].minPWM;
  int maxPWM = servoCalib[servoIndex].maxPWM;
  return map(percent, 0, 100, minPWM, maxPWM);
}

// Main web page with PWM control
void handleRoot() {
  String html = "<!DOCTYPE html><html lang='es'><head><meta charset='UTF-8'>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<title>Control PWM de Gripper - ESP32</title>";
  html += "<style>";
  html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
  html += "body { font-family: Arial, sans-serif; background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); min-height: 100vh; padding: 20px; }";
  html += ".container { max-width: 900px; margin: 0 auto; background: white; border-radius: 15px; box-shadow: 0 10px 30px rgba(0,0,0,0.3); overflow: hidden; }";
  html += ".header { background: linear-gradient(45deg, #2196F3, #21CBF3); color: white; padding: 30px; text-align: center; }";
  html += ".header h1 { font-size: 2.5em; margin-bottom: 10px; }";
  html += ".header p { font-size: 1.1em; opacity: 0.9; }";
  html += ".controls { padding: 30px; }";
  html += ".servo-control { background: #f8f9fa; border-radius: 10px; padding: 20px; margin-bottom: 20px; border-left: 4px solid #2196F3; }";
  html += ".servo-control h3 { color: #333; margin-bottom: 15px; font-size: 1.3em; }";
  html += ".control-group { display: flex; align-items: center; gap: 15px; flex-wrap: wrap; }";
  html += ".slider-container { flex: 1; min-width: 250px; }";
  html += ".slider { width: 100%; height: 8px; border-radius: 5px; background: #ddd; outline: none; -webkit-appearance: none; }";
  html += ".slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #2196F3; cursor: pointer; }";
  html += ".pwm-display { background: #2196F3; color: white; padding: 8px 15px; border-radius: 20px; font-weight: bold; min-width: 80px; text-align: center; font-size: 0.9em; }";
  html += ".percent-display { background: #4CAF50; color: white; padding: 8px 15px; border-radius: 20px; font-weight: bold; min-width: 60px; text-align: center; font-size: 0.9em; }";
  html += ".preset-buttons { display: flex; gap: 8px; margin-top: 15px; flex-wrap: wrap; }";
  html += ".preset-btn { background: #4CAF50; color: white; border: none; padding: 6px 12px; border-radius: 15px; cursor: pointer; font-size: 0.8em; transition: all 0.3s; }";
  html += ".preset-btn:hover { background: #45a049; transform: translateY(-2px); }";
  html += ".preset-btn.pwm { background: #FF9800; }";
  html += ".preset-btn.pwm:hover { background: #F57C00; }";
  html += ".status { background: #e8f5e8; border: 1px solid #4CAF50; border-radius: 10px; padding: 15px; margin-top: 20px; text-align: center; }";
  html += ".emergency-stop { background: #f44336; color: white; border: none; padding: 15px 30px; border-radius: 25px; font-size: 1.1em; font-weight: bold; cursor: pointer; margin: 20px auto; display: block; transition: all 0.3s; }";
  html += ".emergency-stop:hover { background: #d32f2f; transform: scale(1.05); }";
  html += ".info-panel { background: #e3f2fd; border: 1px solid #2196F3; border-radius: 10px; padding: 15px; margin-bottom: 20px; }";
  html += ".info-panel h4 { color: #1976D2; margin-bottom: 10px; }";
  html += ".info-text { font-size: 0.9em; color: #555; }";
  
  // Sequence section styles
  html += ".sequence-section { background: linear-gradient(135deg, #e3f2fd, #f3e5f5); border: 2px solid #2196F3; border-radius: 12px; padding: 20px; margin: 20px 0; box-shadow: 0 4px 12px rgba(33,150,243,0.2); }";
  html += ".sequence-section h2 { color: #1976D2; margin: 0 0 15px 0; text-align: center; font-size: 24px; }";
  html += ".sequence-info { background: rgba(255,255,255,0.8); border-radius: 8px; padding: 15px; margin-bottom: 20px; }";
  html += ".sequence-info ul { margin: 10px 0; padding-left: 20px; }";
  html += ".sequence-info li { margin: 5px 0; color: #424242; }";
  html += ".sequence-buttons { display: grid; grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); gap: 15px; margin-bottom: 15px; }";
  html += ".sequence-btn { padding: 15px 20px; font-size: 16px; font-weight: bold; border: none; border-radius: 8px; cursor: pointer; transition: all 0.3s ease; box-shadow: 0 4px 8px rgba(0,0,0,0.2); }";
  html += ".sequence-btn:hover { transform: translateY(-2px); box-shadow: 0 6px 12px rgba(0,0,0,0.3); }";
  html += ".sequence-btn:active { transform: translateY(0); box-shadow: 0 2px 4px rgba(0,0,0,0.2); }";
  html += ".sequence-btn.full-cycle { background: linear-gradient(135deg, #4CAF50, #388E3C); color: white; }";
  html += ".sequence-btn.forward { background: linear-gradient(135deg, #2196F3, #1976D2); color: white; }";
  html += ".sequence-btn.backward { background: linear-gradient(135deg, #FF9800, #F57C00); color: white; }";
  html += ".sequence-btn.reset { background: linear-gradient(135deg, #9C27B0, #7B1FA2); color: white; }";
  html += ".sequence-status { background: #f5f5f5; border: 2px solid #ddd; border-radius: 8px; padding: 12px; text-align: center; font-weight: bold; color: #666; }";
  
  html += "@media (max-width: 600px) { .control-group { flex-direction: column; align-items: stretch; } .header h1 { font-size: 2em; } .controls { padding: 20px; } }";
  html += "</style></head><body>";
  html += "<div class='container'>";
  html += "<div class='header'>";
  html += "<h1>🏭 AMI Taping Machine</h1>";
  html += "<p>Sistema de Control Industrial PWM - Máquina de Taping Automática</p>";
  html += "</div>";
  html += "<div class='controls'>";
  
  // Info panel
  html += "<div class='info-panel'>";
  html += "<h4>🏭 Control Industrial AMI Taping</h4>";
  html += "<div class='info-text'>Rango PWM: 500-2500μs | Centro: 1500μs | Motores: Feeder, Wrapper, Cutter, Holder, Gripper, Elevator</div>";
  html += "</div>";
  
  // Generate servo controls with motor-specific functionality
  for (int i = 0; i < 6; i++) {
    html += "<div class='servo-control'>";
    html += "<h3>" + servoCalib[i].name + " - Pin " + String(servoPins[i]) + "</h3>";
    html += "<div class='control-group'>";
    html += "<div class='slider-container'>";
    html += "<input type='range' min='" + String(servoCalib[i].minPWM) + "' max='" + String(servoCalib[i].maxPWM) + "' value='" + String(servoPWM[i]) + "' class='slider' id='servo" + String(i) + "' oninput='updateServoPWM(" + String(i) + ", this.value)'>";
    html += "</div>";
    html += "<div class='pwm-display' id='pwm" + String(i) + "'>" + String(servoPWM[i]) + "μs</div>";
    html += "<div class='percent-display' id='percent" + String(i) + "'>" + String(pwmToPercent(servoPWM[i], i)) + "%</div>";
    html += "</div>";
    
    // Motor-specific preset buttons based on AMI functionality
    html += "<div class='preset-buttons'>";
    if (i == 0) { // Feeder
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 500)'>Stop</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 885)'>19mm Tape</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1200)'>Feed Forward</button>";
    } else if (i == 1) { // Wrapper
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1200)'>CCW Slow</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1500)'>Stop</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1650)'>CW Slow</button>";
    } else if (i == 2) { // Cutter
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 500)'>Cut Position</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1500)'>Home Position</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1800)'>Safe Position</button>";
    } else if (i == 3) { // Holder
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1400)'>Release</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1500)'>Home</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1800)'>Hold</button>";
    } else if (i == 4) { // Gripper
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1800)'>Close</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1500)'>Neutral</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 2000)'>Open</button>";
    } else if (i == 5) { // Elevator
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 700)'>Down</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1125)'>Center</button>";
      html += "<button class='preset-btn' onclick='setServoPWM(" + String(i) + ", 1550)'>Up</button>";
    }
    html += "</div>";
    html += "</div>";
  }
  
  // Automated Sequences Section
  html += "<div class='sequence-section'>";
  html += "<h2>🔄 Secuencias Automatizadas</h2>";
  html += "<div class='sequence-info'>";
  html += "<p><strong>Secuencias basadas en AMI_Taping_v9.ino:</strong></p>";
  html += "<ul>";
  html += "<li><strong>Ciclo Completo:</strong> Secuencia principal de taping automático</li>";
  html += "<li><strong>Avance:</strong> Preparación y agarre de manguera</li>";
  html += "<li><strong>Retroceso:</strong> Proceso de wrapping y corte</li>";
  html += "</ul>";
  html += "</div>";
  html += "<div class='sequence-buttons'>";
  html += "<button class='sequence-btn full-cycle' onclick='runSequence(\"fullcycle\")'>🔄 CICLO COMPLETO</button>";
  html += "<button class='sequence-btn forward' onclick='runSequence(\"forward\")'>⬆️ AVANCE</button>";
  html += "<button class='sequence-btn backward' onclick='runSequence(\"backward\")'>⬇️ RETROCESO</button>";
  html += "<button class='sequence-btn reset' onclick='runSequence(\"reset\")'>🏠 RESET POSICIONES</button>";
  html += "</div>";
  html += "<div class='sequence-status' id='sequenceStatus'>Listo para ejecutar secuencias</div>";
  html += "</div>";
  
  html += "<button class='emergency-stop' onclick='emergencyStop()'>🛑 PARADA DE EMERGENCIA</button>";
  html += "<div class='status' id='status'>Sistema PWM listo - Control de precisión activado</div>";
  html += "</div></div>";

  // JavaScript for PWM control
  html += "<script>";
  html += "function updateServoPWM(servoIndex, pwmValue) {";
  html += "let servoRanges = [{min:500,max:2500},{min:500,max:2500},{min:500,max:2500},{min:500,max:2500},{min:500,max:2500},{min:700,max:1550}];";
  html += "let range = servoRanges[servoIndex];";
  html += "document.getElementById('pwm' + servoIndex).textContent = pwmValue + 'μs';";
  html += "let percent = Math.round(((pwmValue - range.min) / (range.max - range.min)) * 100);";
  html += "document.getElementById('percent' + servoIndex).textContent = percent + '%';";
  html += "fetch('/control?servo=' + (servoIndex + 1) + '&pwm=' + pwmValue)";
  html += ".then(response => response.json())";
  html += ".then(data => {";
  html += "if (data.status === 'success') {";
  html += "let servoNames = ['Feeder', 'Wrapper', 'Cutter', 'Holder', 'Gripper', 'Elevator'];";
  html += "document.getElementById('status').textContent = servoNames[servoIndex] + ' → ' + pwmValue + 'μs (' + percent + '%)';";
  html += "document.getElementById('status').style.background = '#e8f5e8';";
  html += "document.getElementById('status').style.borderColor = '#4CAF50';";
  html += "} else {";
  html += "document.getElementById('status').textContent = 'Error: ' + (data.message || 'Comando fallido');";
  html += "document.getElementById('status').style.background = '#ffebee';";
  html += "document.getElementById('status').style.borderColor = '#f44336';";
  html += "}";
  html += "})";
  html += ".catch(error => {";
  html += "document.getElementById('status').textContent = 'Error de conexión: ' + error;";
  html += "document.getElementById('status').style.background = '#ffebee';";
  html += "document.getElementById('status').style.borderColor = '#f44336';";
  html += "});";
  html += "}";
  html += "function setServoPWM(servoIndex, pwmValue) {";
  html += "document.getElementById('servo' + servoIndex).value = pwmValue;";
  html += "updateServoPWM(servoIndex, pwmValue);";
  html += "}";
  html += "function emergencyStop() {";
  html += "if (confirm('¿Estás seguro de que quieres centrar todos los servos?')) {";
  html += "let centerValues = [1500, 1500, 1500, 1500, 1500, 1125];";
  html += "for (let i = 0; i < 6; i++) {";
  html += "setServoPWM(i, centerValues[i]);";
  html += "}";
  html += "document.getElementById('status').textContent = 'PARADA DE EMERGENCIA - Todos los servos centrados';";
  html += "document.getElementById('status').style.background = '#fff3e0';";
  html += "document.getElementById('status').style.borderColor = '#FF9800';";
  html += "}";
  html += "}";
  html += "function updateStatus() {";
  html += "fetch('/status')";
  html += ".then(response => response.json())";
  html += ".then(data => {";
  html += "if (data.servos) {";
  html += "for (let i = 0; i < data.servos.length; i++) {";
  html += "let servo = data.servos[i];";
  html += "document.getElementById('servo' + i).value = servo.pwm;";
  html += "document.getElementById('pwm' + i).textContent = servo.pwm + 'μs';";
  html += "document.getElementById('percent' + i).textContent = servo.percent + '%';";
  html += "}";
  html += "if (document.getElementById('status').style.borderColor !== 'rgb(244, 67, 54)') {";
  html += "document.getElementById('status').textContent = 'Sistema sincronizado - ' + new Date().toLocaleTimeString();";
  html += "document.getElementById('status').style.background = '#e8f5e8';";
  html += "document.getElementById('status').style.borderColor = '#4CAF50';";
  html += "}";
  html += "}";
  html += "})";
  html += ".catch(error => console.log('Status update error:', error));";
  html += "}";
  html += "function runSequence(sequenceType) {";
  html += "if (confirm('¿Ejecutar secuencia ' + sequenceType.toUpperCase() + '?\\n\\nEsta acción moverá múltiples servos automáticamente.')) {";
  html += "document.getElementById('sequenceStatus').textContent = 'Ejecutando secuencia ' + sequenceType + '...';";
  html += "document.getElementById('sequenceStatus').style.background = '#fff3e0';";
  html += "document.getElementById('sequenceStatus').style.borderColor = '#FF9800';";
  html += "fetch('/sequence?type=' + sequenceType)";
  html += ".then(response => response.json())";
  html += ".then(data => {";
  html += "if (data.status === 'success') {";
  html += "document.getElementById('sequenceStatus').textContent = 'Secuencia ' + sequenceType + ' completada exitosamente';";
  html += "document.getElementById('sequenceStatus').style.background = '#e8f5e8';";
  html += "document.getElementById('sequenceStatus').style.borderColor = '#4CAF50';";
  html += "document.getElementById('status').textContent = 'Secuencia ejecutada: ' + data.sequence + ' - ' + new Date().toLocaleTimeString();";
  html += "document.getElementById('status').style.background = '#e8f5e8';";
  html += "document.getElementById('status').style.borderColor = '#4CAF50';";
  html += "} else {";
  html += "document.getElementById('sequenceStatus').textContent = 'Error en secuencia: ' + (data.message || 'Fallo desconocido');";
  html += "document.getElementById('sequenceStatus').style.background = '#ffebee';";
  html += "document.getElementById('sequenceStatus').style.borderColor = '#f44336';";
  html += "}";
  html += "})";
  html += ".catch(error => {";
  html += "document.getElementById('sequenceStatus').textContent = 'Error de conexión: ' + error;";
  html += "document.getElementById('sequenceStatus').style.background = '#ffebee';";
  html += "document.getElementById('sequenceStatus').style.borderColor = '#f44336';";
  html += "});";
  html += "}";
  html += "}";
  html += "setInterval(updateStatus, 5000);";
  html += "updateStatus();";
  html += "</script>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

// Handle servo control via PWM
void handleServoControl() {
  if (server.hasArg("servo")) {
    int servoIndex = server.arg("servo").toInt() - 1; // Convert to 0-based index
    
    // Validate servo index
    if (servoIndex < 0 || servoIndex >= 6) {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid servo index\"}");
      return;
    }
    
    // Handle PWM control
    if (server.hasArg("pwm")) {
      int pwmValue = server.arg("pwm").toInt();
      if (pwmValue >= PWM_MIN && pwmValue <= PWM_MAX) {
        moveServoPWM(servoIndex, pwmValue);
        server.send(200, "application/json", "{\"status\":\"success\",\"servo\":" + String(servoIndex + 1) + ",\"pwm\":" + String(pwmValue) + "}");
      } else {
        server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"PWM out of range (500-2500)\"}");
      }
    }
    // Handle percentage control
     else if (server.hasArg("percent")) {
       int percent = server.arg("percent").toInt();
       if (percent >= 0 && percent <= 100) {
         int pwmValue = percentToPWM(percent, servoIndex);
         moveServoPWM(servoIndex, pwmValue);
         server.send(200, "application/json", "{\"status\":\"success\",\"servo\":" + String(servoIndex + 1) + ",\"percent\":" + String(percent) + ",\"pwm\":" + String(pwmValue) + "}");
       } else {
         server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Percentage out of range (0-100)\"}");
       }
     }
     // Handle legacy position control (convert to PWM)
     else if (server.hasArg("position")) {
       int position = server.arg("position").toInt();
       if (position >= 0 && position <= 180) {
         int percent = map(position, 0, 180, 0, 100);
         int pwmValue = percentToPWM(percent, servoIndex);
         moveServoPWM(servoIndex, pwmValue);
         server.send(200, "application/json", "{\"status\":\"success\",\"servo\":" + String(servoIndex + 1) + ",\"position\":" + String(position) + ",\"pwm\":" + String(pwmValue) + "}");
       } else {
         server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Position out of range (0-180)\"}");
       }
     } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing control parameter (pwm, percent, or position)\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing servo parameter\"}");
  }
}

// Handle calibration requests
void handleCalibration() {
  if (server.hasArg("servo") && server.hasArg("min") && server.hasArg("max")) {
    int servoIndex = server.arg("servo").toInt() - 1;
    int minPWM = server.arg("min").toInt();
    int maxPWM = server.arg("max").toInt();
    
    if (servoIndex >= 0 && servoIndex < 6 && minPWM >= 500 && maxPWM <= 2500 && minPWM < maxPWM) {
      servoCalib[servoIndex].minPWM = minPWM;
      servoCalib[servoIndex].maxPWM = maxPWM;
      servoCalib[servoIndex].centerPWM = (minPWM + maxPWM) / 2;
      
      // Re-attach servo with new calibration
      servos[servoIndex].detach();
      delay(50);
      servos[servoIndex].attach(servoPins[servoIndex], minPWM, maxPWM);
      
      Serial.print("Calibración actualizada para ");
      Serial.print(servoCalib[servoIndex].name);
      Serial.print(": Min=");
      Serial.print(minPWM);
      Serial.print("μs, Max=");
      Serial.print(maxPWM);
      Serial.print("μs, Centro=");
      Serial.print(servoCalib[servoIndex].centerPWM);
      Serial.println("μs");
      
      server.send(200, "application/json", "{\"status\":\"success\",\"servo\":" + String(servoIndex + 1) + ",\"minPWM\":" + String(minPWM) + ",\"maxPWM\":" + String(maxPWM) + ",\"centerPWM\":" + String(servoCalib[servoIndex].centerPWM) + "}");
    } else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid calibration parameters\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Missing calibration parameters\"}");
  }
}

// Handle status requests
void handleStatus() {
  String json = "{\"servos\":[";
  for (int i = 0; i < 6; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"id\":" + String(i + 1) + ",";
    json += "\"name\":\"" + servoCalib[i].name + "\",";
    json += "\"pin\":" + String(servoPins[i]) + ",";
    json += "\"pwm\":" + String(servoPWM[i]) + ",";
    json += "\"percent\":" + String(pwmToPercent(servoPWM[i], i)) + ",";
    json += "\"minPWM\":" + String(servoCalib[i].minPWM) + ",";
    json += "\"maxPWM\":" + String(servoCalib[i].maxPWM) + ",";
    json += "\"centerPWM\":" + String(servoCalib[i].centerPWM);
    json += "}";
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// Handle automated sequences
void handleSequence() {
  if (server.hasArg("type")) {
    String sequenceType = server.arg("type");
    sequenceType.toLowerCase();
    
    Serial.println("🔄 Ejecutando secuencia: " + sequenceType);
    
    if (sequenceType == "fullcycle") {
      executeFullCycleSequence();
      server.send(200, "application/json", "{\"status\":\"success\",\"sequence\":\"Full Cycle\",\"message\":\"Secuencia de ciclo completo ejecutada\"}");
    } 
    else if (sequenceType == "forward") {
      executeForwardSequence();
      server.send(200, "application/json", "{\"status\":\"success\",\"sequence\":\"Forward\",\"message\":\"Secuencia de avance ejecutada\"}");
    } 
    else if (sequenceType == "backward") {
      executeBackwardSequence();
      server.send(200, "application/json", "{\"status\":\"success\",\"sequence\":\"Backward\",\"message\":\"Secuencia de retroceso ejecutada\"}");
    } 
    else if (sequenceType == "reset") {
      executeResetSequence();
      server.send(200, "application/json", "{\"status\":\"success\",\"sequence\":\"Reset\",\"message\":\"Posiciones reseteadas a home\"}");
    } 
    else {
      server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Tipo de secuencia no válido\"}");
    }
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Parámetro 'type' requerido\"}");
  }
}

// Automated Sequence Functions based on AMI_Taping_v9.ino

// Full Cycle Sequence: step7() -> step1() -> step6() -> step4()
void executeFullCycleSequence() {
  Serial.println("🔄 Iniciando secuencia CICLO COMPLETO");
  
  // Step 7: Holder to home position
  Serial.println("  Step 7: Holder a posición home");
  moveServoPWM(3, 1500); // Holder home
  delay(500);
  
  // Step 1: Feeder forward movement
  Serial.println("  Step 1: Feeder avance");
  moveServoPWM(0, 1200); // Feeder forward
  delay(1000);
  moveServoPWM(0, 1500); // Feeder stop
  delay(500);
  
  // Step 6: Gripper operations
  Serial.println("  Step 6: Operaciones de gripper");
  moveServoPWM(4, 2000); // Gripper open
  delay(500);
  moveServoPWM(4, 1800); // Gripper close
  delay(500);
  
  // Step 4: Cutter operation
  Serial.println("  Step 4: Operación de corte");
  moveServoPWM(2, 500);  // Cutter cut position
  delay(1000);
  moveServoPWM(2, 1500); // Cutter home position
  delay(500);
  
  Serial.println("✅ Secuencia CICLO COMPLETO completada");
}

// Forward Sequence: step10() -> step8() -> step7() -> step12() -> step9()
void executeForwardSequence() {
  Serial.println("🔄 Iniciando secuencia AVANCE");
  
  // Step 10: Elevator down
  Serial.println("  Step 10: Elevator abajo");
  moveServoPWM(5, 700); // Elevator down
  delay(500);
  
  // Step 8: Gripper close
  Serial.println("  Step 8: Gripper cerrar");
  moveServoPWM(4, 1800); // Gripper close
  delay(500);
  
  // Step 7: Holder home
  Serial.println("  Step 7: Holder home");
  moveServoPWM(3, 1500); // Holder home
  delay(500);
  
  // Step 12: Additional holder operation
  Serial.println("  Step 12: Operación adicional holder");
  moveServoPWM(3, 1800); // Holder hold position
  delay(500);
  
  // Step 9: Gripper open
  Serial.println("  Step 9: Gripper abrir");
  moveServoPWM(4, 2000); // Gripper open
  delay(500);
  
  Serial.println("✅ Secuencia AVANCE completada");
}

// Backward Sequence: step2() -> step3() -> step4() -> step6() -> step5()
void executeBackwardSequence() {
  Serial.println("🔄 Iniciando secuencia RETROCESO");
  
  // Step 2: Wrapper CCW 1/4 spin
  Serial.println("  Step 2: Wrapper CCW 1/4 vuelta");
  moveServoPWM(1, 1200); // Wrapper CCW slow
  delay(2000);
  moveServoPWM(1, 1500); // Wrapper stop
  delay(500);
  
  // Step 3: Wrapper CW 1/2 spin
  Serial.println("  Step 3: Wrapper CW 1/2 vuelta");
  moveServoPWM(1, 1650); // Wrapper CW slow
  delay(4000);
  moveServoPWM(1, 1500); // Wrapper stop
  delay(500);
  
  // Step 4: Cutter operation
  Serial.println("  Step 4: Operación de corte");
  moveServoPWM(2, 500);  // Cutter cut position
  delay(1000);
  moveServoPWM(2, 1500); // Cutter home position
  delay(500);
  
  // Step 6: Gripper operations
  Serial.println("  Step 6: Operaciones de gripper");
  moveServoPWM(4, 2000); // Gripper open
  delay(500);
  moveServoPWM(4, 1800); // Gripper close
  delay(500);
  
  // Step 5: Wrapper 5 spins CCW
  Serial.println("  Step 5: Wrapper 5 vueltas CCW");
  moveServoPWM(1, 1200); // Wrapper CCW slow
  delay(10000); // Extended time for 5 rotations
  moveServoPWM(1, 1500); // Wrapper stop
  delay(500);
  
  Serial.println("✅ Secuencia RETROCESO completada");
}

// Reset Sequence: All servos to home/center positions
void executeResetSequence() {
  Serial.println("🔄 Iniciando secuencia RESET");
  
  Serial.println("  Moviendo todos los servos a posición home...");
  
  // Move all servos to safe home positions
  moveServoPWM(0, 1500); // Feeder stop/center
  delay(200);
  moveServoPWM(1, 1500); // Wrapper stop
  delay(200);
  moveServoPWM(2, 1500); // Cutter home position
  delay(200);
  moveServoPWM(3, 1500); // Holder home
  delay(200);
  moveServoPWM(4, 1500); // Gripper neutral
  delay(200);
  moveServoPWM(5, 1125); // Elevator center
  delay(500);
  
  Serial.println("✅ Secuencia RESET completada - Todos los servos en posición home");
}