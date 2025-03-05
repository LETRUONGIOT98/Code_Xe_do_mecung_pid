#include <NewPing.h>
#define LEFT_TRIG_PIN 2
#define LEFT_ECHO_PIN 3
#define FRONT_TRIG_PIN 4
#define FRONT_ECHO_PIN 5
#define RIGHT_TRIG_PIN 6
#define RIGHT_ECHO_PIN 7
#define MOTOR1_ENA 8
#define MOTOR1_IN1 9
#define MOTOR1_IN2 10
#define MOTOR2_ENB 11
#define MOTOR2_IN3 12
#define MOTOR2_IN4 13
#define MAX_DISTANCE 200

NewPing leftSonar(LEFT_TRIG_PIN, LEFT_ECHO_PIN, MAX_DISTANCE);
NewPing frontSonar(FRONT_TRIG_PIN, FRONT_ECHO_PIN, MAX_DISTANCE);
NewPing rightSonar(RIGHT_TRIG_PIN, RIGHT_ECHO_PIN, MAX_DISTANCE);

double Kp = 1.5; // Hệ số tỷ lệ Proportional
double Ki = 1.0; // Hệ số tích phân Integral
double Kd = 1.2; // Hệ số đạo hàm Derivative

double setpoint = 10.0; // Khoảng cách mục tiêu trong đơn vị centimet
double input, output, error, lastError;
double integral, derivative;

void setup() {
  pinMode(MOTOR1_ENA, OUTPUT);
  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_ENB, OUTPUT);
  pinMode(MOTOR2_IN3, OUTPUT);
  pinMode(MOTOR2_IN4, OUTPUT);
  
  Serial.begin(9600);
  lastError = 0;
}

void loop() {
  // Đọc giá trị từ các cảm biến
  double leftDistance = leftSonar.ping_cm();
  double frontDistance = frontSonar.ping_cm();
  double rightDistance = rightSonar.ping_cm();

  // Tính toán sai số (error)
  error = setpoint - frontDistance;
  
  // Thuật toán tỷ lệ Proportional
  double pTerm = Kp * error;

  // Thuật toán tích phân Integral
  integral += error;
  double iTerm = Ki * integral;

  // Thuật toán đạo hàm Derivative
  derivative = error - lastError;
  double dTerm = Kd * derivative;

  // Tính toán giá trị điều khiển PID
  output = pTerm + iTerm + dTerm;

  // Cập nhật tốc độ động cơ trái và phải
  double leftSpeed = 100 + output;
  double rightSpeed = 100 - output;

  // Đảm bảo rằng tốc độ động cơ nằm trong giới hạn cho phép
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Áp dụng điều khiển động cơ
  analogWrite(MOTOR1_ENA, leftSpeed);
  analogWrite(MOTOR2_ENB, rightSpeed);
  digitalWrite(MOTOR1_IN1, leftSpeed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR1_IN2, leftSpeed < 0 ? HIGH : LOW);
  digitalWrite(MOTOR2_IN3, rightSpeed > 0 ? HIGH : LOW);
  digitalWrite(MOTOR2_IN4, rightSpeed < 0 ? HIGH : LOW);

  // In giá trị từ cảm biến và điều khiển ra màn hình
  Serial.print("Trái: ");
  Serial.print(leftDistance);
  Serial.print(" cm, Trước: ");
  Serial.print(frontDistance);
  Serial.print(" cm, Phải: ");
  Serial.print(rightDistance);
  Serial.print(" cm, Điều khiển: ");
  Serial.println(output);

  // Lưu sai số hiện tại để sử dụng cho vòng lặp tiếp theo
  lastError = error;
  
  delay(100);
}
