int send_signal_1 = D1;    // start signal -> dobot 1
int send_signal_2 = D0;    // start signal -> dobot 2

int ir_sensor_1 = A7;      // IR sensor that triggers dobot 1
int ir_sensor_2 = A6;      // IR sensor that triggers dobot 2

int IR_THRESHOLD = 50;     // tune if needed (was 50 in your working setup)

void setup() {
  pinMode(ir_sensor_1, INPUT);
  pinMode(ir_sensor_2, INPUT);
  pinMode(send_signal_1, OUTPUT);
  pinMode(send_signal_2, OUTPUT);
  digitalWrite(send_signal_1, LOW);
  digitalWrite(send_signal_2, LOW);

  Serial.begin(9600);
}

void loop() {
  int ir_val_1 = analogRead(ir_sensor_1);
  int ir_val_2 = analogRead(ir_sensor_2);

  Serial.print("IR1=");
  Serial.print(ir_val_1);
  Serial.print("  IR2=");
  Serial.println(ir_val_2);

  if (ir_val_1 <= IR_THRESHOLD) {
    digitalWrite(send_signal_1, HIGH);
  } else {
    digitalWrite(send_signal_1, LOW);
  }

  if (ir_val_2 <= IR_THRESHOLD) {
    digitalWrite(send_signal_2, HIGH);
  } else {
    digitalWrite(send_signal_2, LOW);
  }
}