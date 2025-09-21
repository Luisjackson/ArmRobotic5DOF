//#include <DynamixelShield.h>

// using namespace ControlTableItem; 

// DynamixelShield dxl; 

// const int JOINT_COUNT = 5;

// const uint8_t DXL_IDS[JOINT_COUNT] = {1, 2, 3, 4, 5}; // IDs dos seus servos nesta ordem

// char serial_buffer[100];
// int buffer_pos = 0;


// unsigned long last_status_update_time = 0;
// const int STATUS_UPDATE_INTERVAL_MS = 100; // Envia o estado 10 vezes por segundo (10 Hz)


// void setup() {

//   Serial.begin(115200);
//   dxl.begin(1000000); 
//   dxl.setPortProtocolVersion(1.0);

//   for (int i = 0; i < JOINT_COUNT; i++) {
//     if (dxl.ping(DXL_IDS[i])) {
//       dxl.setOperatingMode(DXL_IDS[i], OP_POSITION);
//       dxl.torqueOn(DXL_IDS[i]);
//     }
//   }
// }

// void loop() {

//   process_serial_commands();

//   publish_current_state();
// }

// void process_serial_commands() {
//   while (Serial.available() > 0) {
//     char in_char = Serial.read();

//     if (in_char == '\n') {
//       serial_buffer[buffer_pos] = '\0'; // Finaliza a string

//       if (serial_buffer[0] == 'P' && serial_buffer[1] == ' ') {
//         int p[JOINT_COUNT];
//         int items_parsed = sscanf(serial_buffer + 2, "%d %d %d %d %d", &p[0], &p[1], &p[2], &p[3], &p[4]);

//         if (items_parsed == JOINT_COUNT) {
//           for (int i = 0; i < JOINT_COUNT; i++) {
//             dxl.setGoalPosition(DXL_IDS[i], p[i]);
//           }
//         }
//       }
      
//       buffer_pos = 0; // Reseta o buffer para o próximo comando.
//       break; // Sai do while para dar tempo a outras tarefas.
//     } 
//     else {
//       if (buffer_pos < sizeof(serial_buffer) - 1) {
//         serial_buffer[buffer_pos++] = in_char;
//       }
//     }
//   }
// }

// void publish_current_state() {
//   if (millis() - last_status_update_time >= STATUS_UPDATE_INTERVAL_MS) {
//     last_status_update_time = millis(); // Marca o tempo do envio atual.

//     Serial.print("S ");
//     for (int i = 0; i < JOINT_COUNT; i++) {
//       Serial.print(dxl.getPresentPosition(DXL_IDS[i]));
//       if (i < JOINT_COUNT - 1) {
//         Serial.print(" ");
//       }
//     }
//     Serial.println(); // Envia a quebra de linha para finalizar a mensagem.
//   }
// }

