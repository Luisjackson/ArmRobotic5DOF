// // --- Firmware Robusto para Braço Robótico com Dynamixel e ROS 2 ---
// // Este código usa a biblioteca DynamixelShield e uma comunicação serial não-bloqueante.

// #include <DynamixelShield.h>

// // Usar o namespace da biblioteca para facilitar a escrita
// using namespace ControlTableItem; 

// // --- Configuração ---
// // Inicializa o objeto da shield Dynamixel.
// DynamixelShield dxl; 

// const int JOINT_COUNT = 5;
// // Ordem das juntas DEVE ser a mesma do C++ e dos arquivos YAML: 
// // waist_joint, shoulder_pitch_joint, elbow_pitch_joint, wrist_pitch_joint, gripper_finger_joint
// const uint8_t DXL_IDS[JOINT_COUNT] = {1, 2, 3, 4, 5}; // IDs dos seus servos nesta ordem

// // Variáveis para comunicação serial não-bloqueante
// char serial_buffer[100];
// int buffer_pos = 0;

// // Variáveis para controle de tempo do envio de estado
// unsigned long last_status_update_time = 0;
// const int STATUS_UPDATE_INTERVAL_MS = 100; // Envia o estado 10 vezes por segundo (10 Hz)

// // --- Fim da Configuração ---

// void setup() {
//   // Inicia a comunicação serial com o PC (ROS)
//   Serial.begin(115200);

//   // Inicia a comunicação com os servos Dynamixel na velocidade correta (1 Mbps)
//   dxl.begin(1000000); 
//   dxl.setPortProtocolVersion(1.0);

//   // Configura cada servo
//   for (int i = 0; i < JOINT_COUNT; i++) {
//     if (dxl.ping(DXL_IDS[i])) {
//       dxl.setOperatingMode(DXL_IDS[i], OP_POSITION);
//       dxl.torqueOn(DXL_IDS[i]);
//     }
//   }
// }

// void loop() {
//   // O loop principal agora executa duas tarefas continuamente, sem bloqueios.
  
//   // 1. Processa qualquer dado que tenha chegado do ROS 2.
//   process_serial_commands();

//   // 2. Envia o estado atual do braço para o ROS 2 em intervalos regulares.
//   publish_current_state();
// }

// void process_serial_commands() {
//   while (Serial.available() > 0) {
//     char in_char = Serial.read();

//     // Se recebermos uma quebra de linha, o comando está completo.
//     if (in_char == '\n') {
//       serial_buffer[buffer_pos] = '\0'; // Finaliza a string

//       // Verifica se o comando é de Posição ("P ...")
//       if (serial_buffer[0] == 'P' && serial_buffer[1] == ' ') {
//         int p[JOINT_COUNT];
//         // sscanf é uma forma muito rápida e segura de extrair números de uma string.
//         int items_parsed = sscanf(serial_buffer + 2, "%d %d %d %d %d", &p[0], &p[1], &p[2], &p[3], &p[4]);
        
//         // Se conseguimos ler todos os 5 valores, move os servos.
//         if (items_parsed == JOINT_COUNT) {
//           for (int i = 0; i < JOINT_COUNT; i++) {
//             dxl.setGoalPosition(DXL_IDS[i], p[i]);
//           }
//         }
//       }
      
//       buffer_pos = 0; // Reseta o buffer para o próximo comando.
//       break; // Sai do while para dar tempo a outras tarefas.
//     } 
//     // Se não for quebra de linha, adiciona o caractere ao buffer.
//     else {
//       if (buffer_pos < sizeof(serial_buffer) - 1) {
//         serial_buffer[buffer_pos++] = in_char;
//       }
//     }
//   }
// }

// void publish_current_state() {
//   // Verifica se já passou tempo suficiente desde o último envio.
//   if (millis() - last_status_update_time >= STATUS_UPDATE_INTERVAL_MS) {
//     last_status_update_time = millis(); // Marca o tempo do envio atual.

//     // Monta e envia a string de estado "S pos1 pos2 ..."
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

