#include <Servo.h>

Servo continuousServo;  // 연속 회전 서보 모터 객체 생성

void setup() {
  continuousServo.attach(5);  // 서보 모터를 아두이노의 5번 핀에 연결
  continuousServo.write(90);  // 초기 상태는 정지
  delay(1000);  // 초기화 후 잠시 대기
}

void loop() {
  // 속도를 5씩 증가시킴
  for (int speed = 90; speed <= 180; speed += 5) {
    continuousServo.write(speed);
    delay(1000);  // 각 속도 단계에서 잠시 대기 (0.1초)
  }

  // 속도를 5씩 감소시킴
  for (int speed = 180; speed >= 0; speed -= 5) {
    continuousServo.write(speed);
    delay(1000);  // 각 속도 단계에서 잠시 대기 (0.1초)
  }

  // 다시 정지
  continuousServo.write(90);
  delay(3000);  // 1초 동안 정지
}
