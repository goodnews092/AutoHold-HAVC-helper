/**
 * [프로젝트: ESP32-C3 SuperMini - 스마트 CAN 인젝션 & HVAC 리포터 + 비프음 제어]
 * - 역할 1: CAN 상태 모니터링 및 변경사항 발생 시 능동 제어
 * - 역할 2: HVAC(공조), 시트, 핸들열선 상태를 감지하여 S3(Master)로 보고 (0xA2)
 * - 수정 사항 (2026-01-23):
 * 1. g_beepEnabled 변수 volatile 선언 (멀티태스킹 데이터 무결성 강화)
 * 2. NVS 변수 로딩 로직을 Task 내부에서 setup()으로 이동 (초기화 시점 보장)
 * * - 추가 수정 사항 (비프음 제어 로직 변경):
 * 1. 0x609 첫 바이트 0x19 -> 비프음 ON (True)
 * 2. 0x609 첫 바이트 0x00 -> 비프음 OFF (False)
 * 3. 0x630 확인 로직 제거
 * 4. NVS 저장 최적화: 상태가 실제로 변경될 때만 prefs.putBool 호출
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <Preferences.h> 

// ===============================================================
// [사용자 설정 변수]
// ===============================================================
const bool ENABLE_OP_MODE_LOGGING = false; 

// [중요] CAN 통신 속도 설정 (C-CAN: 500000 권장)
#define CAN_BITRATE   500000 

// ===============================================================
// 1. 핀 맵핑 (ESP32-C3 SuperMini)
// ===============================================================
#define PIN_CAN_TX    20  
#define PIN_CAN_RX    21  

#define PIN_S3_RX     1   
#define PIN_S3_TX     0   

// ===============================================================
// 2. 설정 상수 & CAN ID
// ===============================================================
#define S3_BAUDRATE   115200 

// 시트 상태 수신용 ID
#define ID_SEAT_DRIVER    0x496
#define ID_SEAT_PASSENGER 0x475
#define ID_SEAT_RL        0x438
#define ID_SEAT_RR        0x453

// HVAC 및 핸들열선 상태 수신용 ID
#define ID_HVAC_AC        0x382 // AC Status
#define ID_HVAC_SYNC      0x383 // Sync Status
#define ID_HVAC_MODE      0x384 // DriverOnly, Heat
#define ID_STEER_HEAT     0x418 // Steering Wheel Heat

// [제어 ID]
#define ID_SEAT_CMD       0x4A2 // 시트 제어
#define ID_HVAC_CMD_1     0x41D // Driver Only, Heat
#define ID_HVAC_CMD_2     0x49F // AC
#define ID_HVAC_CMD_3     0x4A0 // SYNC

// [핸들 열선 트리거 ID]
#define ID_STEER_TRIGGER  0x4CA 

// [비프음 관련 ID]
#define ID_BEEP_TRIGGER   0x609 // 비프음 설정 제어 ID (이것만 사용)
#define ID_BEEP_CONFIRM   0x630 // 인젝션용 ID (로직 감지에서는 제외됨)

// ===============================================================
// 3. 전역 변수 및 구조체
// ===============================================================
enum Mode { MODE_ANALYZER, MODE_OPERATION };
volatile Mode currentMode = MODE_OPERATION;

// 핸들 열선 인젝션 대기 플래그
volatile bool g_waitSteerInjection = false;

// [비프음 설정 상태] volatile 추가하여 최적화 방지 및 즉시 반영 보장
volatile bool g_beepEnabled = true; 

// [수신용] S3에서 오는 제어 명령 (0xA1)
struct __attribute__((packed)) HvacData {
    uint8_t packet_type; // 0xA1
    uint8_t fl_heat; uint8_t fr_heat;
    uint8_t fl_vent; uint8_t fr_vent;
    uint8_t steer_heat;
    uint8_t rl_heat; uint8_t rr_heat;
    uint8_t driver_only; uint8_t ac;
    uint8_t heat; uint8_t sync;
};

// [송신용] S3로 보내는 상태 보고 (0xA2)
struct __attribute__((packed)) FeedbackData {
    uint8_t packet_type; // 0xA2
    uint8_t fl_heat; uint8_t fr_heat; uint8_t fl_vent; uint8_t fr_vent;
    uint8_t steer_heat;
    uint8_t rl_heat; uint8_t rr_heat;
    uint8_t driver_only; uint8_t ac; uint8_t heat; uint8_t sync;
};

// [비프음 큐 데이터 구조체] ID와 첫번째 데이터 바이트 전달용
struct BeepInfo {
    uint32_t id;
    uint8_t data0;
};

// 현재 차량 상태 저장소
FeedbackData g_currentCarState; 
SemaphoreHandle_t xMutex_State; 
QueueHandle_t xQueue_Hvac;
QueueHandle_t xQueue_Beep; 

// NVS 객체
Preferences prefs;

// ===============================================================
// 4. 유틸리티 함수
// ===============================================================

void printRawCanData(twai_message_t message) {
    if (message.extd) Serial.print("T");
    else Serial.print("t");

    char idBuffer[9];
    if (message.extd) sprintf(idBuffer, "%08X", (unsigned int)message.identifier);
    else sprintf(idBuffer, "%03X", (unsigned int)message.identifier);
    Serial.print(idBuffer);

    Serial.print(message.data_length_code);

    for (int i = 0; i < message.data_length_code; i++) {
        char byteBuffer[3];
        sprintf(byteBuffer, "%02X", message.data[i]);
        Serial.print(byteBuffer);
    }
    Serial.print("\r");
}

void checkAndRecoverCAN() {
    twai_status_info_t status_info;
    twai_get_status_info(&status_info);
    if (status_info.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery();
    else if (status_info.state == TWAI_STATE_STOPPED) twai_start();
}

void sendRawFrame(uint16_t id, const uint8_t* data) {
    twai_message_t tx_msg;
    tx_msg.identifier = id;
    tx_msg.extd = 0;
    tx_msg.data_length_code = 8;
    tx_msg.rtr = 0;
    memcpy(tx_msg.data, data, 8);
    twai_transmit(&tx_msg, 0);
}

// [보고] S3(Master)로 현재 상태 보고 함수
void reportStatusToMaster() {
    if (xSemaphoreTake(xMutex_State, pdMS_TO_TICKS(10)) == pdTRUE) {
        g_currentCarState.packet_type = 0xA2;
        Serial1.write((uint8_t*)&g_currentCarState, sizeof(FeedbackData));
        xSemaphoreGive(xMutex_State);
    }
}

// --------------------------------------------------------------------------
// [개선된 인젝션 함수: Interleaving]
// 비프음이 켜져있으면 타겟 명령과 비프 명령을 번갈아 전송하여 딜레이 최소화
// --------------------------------------------------------------------------

// 비프음 데이터 정의
const uint8_t BEEP_ON_DATA[8]  = {0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
const uint8_t BEEP_OFF_DATA[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 통합 인젝션 함수 (Target ID, Target Data, Target Termination Data)
void injectInterleavedBurst(uint16_t targetId, const uint8_t* targetCmd, const uint8_t* targetTerm) {
    // 1. 명령 프레임 전송 (3회)
    for(int i = 0; i < 3; i++) {
        // (1) 타겟 장치 제어 패킷 전송
        sendRawFrame(targetId, targetCmd);
        
        // (2) 비프음 설정이 켜져있으면 사이에 끼워넣기
        if (g_beepEnabled) {
            sendRawFrame(ID_BEEP_CONFIRM, BEEP_ON_DATA);
        }
        
        vTaskDelay(pdMS_TO_TICKS(90));
    }

    // 2. 종료 프레임 전송 (3회)
    for(int i = 0; i < 3; i++) {
        // (1) 타겟 장치 종료 패킷 전송
        sendRawFrame(targetId, targetTerm);
        
        // (2) 비프음 종료 패킷 끼워넣기
        if (g_beepEnabled) {
            sendRawFrame(ID_BEEP_CONFIRM, BEEP_OFF_DATA);
        }
        
        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

// [단독 비프음 인젝션] - 핸들 열선 등에서 사용
void injectBeepBurstOnly() {
    if (!g_beepEnabled) return;

    for(int i = 0; i < 3; i++) {
        sendRawFrame(ID_BEEP_CONFIRM, BEEP_ON_DATA);
        vTaskDelay(pdMS_TO_TICKS(90));
    }
    for(int i = 0; i < 3; i++) {
        sendRawFrame(ID_BEEP_CONFIRM, BEEP_OFF_DATA);
        vTaskDelay(pdMS_TO_TICKS(90));
    }
}

// [핸들 열선 트리거 인젝션] - 0x4CA 감지 시 canTask에서 호출
void injectSteerTriggerBurst() {
    const uint8_t STEER_CMD[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00};
    for(int i = 0; i < 4; i++) {
        sendRawFrame(ID_STEER_TRIGGER, STEER_CMD);
        vTaskDelay(pdMS_TO_TICKS(5)); 
    }
}

// 니블 생성 및 파싱 함수들
uint8_t getFrontNibble(uint8_t heat, uint8_t vent) {
    if (heat == 3) return 0x8; if (heat == 2) return 0x7; if (heat == 1) return 0x6;
    if (vent == 3) return 0x5; if (vent == 2) return 0x4; if (vent == 1) return 0x3;
    return 0x2; 
}
uint8_t getRearNibble(uint8_t heat) {
    if (heat >= 2) return 0x8; if (heat == 1) return 0x6;
    return 0x2; 
}
void parseFrontSeatStatus(uint8_t val, uint8_t &heatOut, uint8_t &ventOut) {
    heatOut = 0; ventOut = 0;
    if (val == 0x46) heatOut = 3; else if (val == 0x3E) heatOut = 2; else if (val == 0x36) heatOut = 1;
    else if (val == 0x2E) ventOut = 3; else if (val == 0x26) ventOut = 2; else if (val == 0x1E) ventOut = 1;
}
void parseRearSeatStatus(uint8_t val, uint8_t &heatOut) {
    heatOut = 0;
    if (val == 0x41) heatOut = 2; else if (val == 0x31) heatOut = 1;
}

// ===============================================================
// 6. FreeRTOS 태스크
// ===============================================================

// 비프음 모니터링 및 NVS 저장 태스크
void beepMonitorTask(void *parameter) {
    BeepInfo rxInfo;

    while (1) {
        // [수정] BeepInfo 구조체(ID + data[0]) 수신
        if (xQueueReceive(xQueue_Beep, &rxInfo, portMAX_DELAY) == pdTRUE) {
            
            if (rxInfo.id == ID_BEEP_TRIGGER) {
                // [조건 1] 첫 번째 바이트가 0x19이면 True로 설정
                if (rxInfo.data0 == 0x19) {
                    // [중요] 현재 상태가 false일 때만 저장 (불필요한 쓰기 방지)
                    if (g_beepEnabled == false) {
                        g_beepEnabled = true;
                        prefs.putBool("beep", true);
                        Serial.println("[BEEP] Mode Set: TRUE (Saved)");
                    }
                } 
                // [조건 2] 첫 번째 바이트가 0x00이면 False로 설정
                else if (rxInfo.data0 == 0x00) {
                    // [중요] 현재 상태가 true일 때만 저장 (불필요한 쓰기 방지)
                    if (g_beepEnabled == true) {
                        g_beepEnabled = false;
                        prefs.putBool("beep", false);
                        Serial.println("[BEEP] Mode Set: FALSE (Saved)");
                    }
                }
                // [조건 3] 0x00도 0x19도 아니면 아무 동작 안 함 (기존 상태 유지)
            }
        }
    }
}

void s3LinkTask(void *parameter) {
    uint8_t rxBuffer[sizeof(HvacData)];
    String lineBuffer = ""; 

    while (1) {
        if (Serial1.available()) {
            uint8_t c = Serial1.read(); 

            if (c == 0xA1) {
                lineBuffer = ""; 
                rxBuffer[0] = 0xA1;
                unsigned long startWait = millis();
                int bytesNeeded = sizeof(HvacData) - 1;

                while (Serial1.available() < bytesNeeded && (millis() - startWait < 50)) {
                    vTaskDelay(1);
                }

                if (Serial1.available() >= bytesNeeded) {
                    Serial1.readBytes(&rxBuffer[1], bytesNeeded);
                    xQueueSend(xQueue_Hvac, (HvacData*)rxBuffer, 0);
                }
            } 
            else if (c == 'S') {
                reportStatusToMaster();
            }
            else {
                if (c == '\n') {
                    lineBuffer.trim();
                    if (lineBuffer.length() > 0) {
                        if (lineBuffer.startsWith("A")) {
                            currentMode = MODE_ANALYZER;
                            Serial1.println("ACK:A");
                        } else if (lineBuffer.startsWith("O")) {
                            currentMode = MODE_OPERATION;
                            Serial1.println("ACK:O");
                        }
                    }
                    lineBuffer = ""; 
                } 
                else if (lineBuffer.length() < 32) {
                    lineBuffer += (char)c;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

void canTask(void *parameter) {
    twai_message_t message;
    
    // 중복 전송 방지용 상태 저장
    uint8_t last_fl_h = 0, last_fl_v = 0, last_fr_h = 0, last_fr_v = 0;
    uint8_t last_rl_h = 0, last_rr_h = 0;
    uint8_t last_ac = 0, last_sync = 0, last_drv = 0, last_heat = 0, last_steer = 0;

    while (1) {
        checkAndRecoverCAN();
        
        if (twai_receive(&message, pdMS_TO_TICKS(5)) == ESP_OK) {
            
            // [트리거 인젝션] 0x4CA가 발견되면 즉시 발사
            if (message.identifier == ID_STEER_TRIGGER) {
                if (g_waitSteerInjection) {
                    injectSteerTriggerBurst();
                    g_waitSteerInjection = false; 
                }
            }

            // [수정] 0x609 감지 시 데이터 첫 바이트를 포함하여 큐로 전송
            // (ID_BEEP_CONFIRM 0x630 로직은 제거됨)
            if (message.identifier == ID_BEEP_TRIGGER) {
                BeepInfo info;
                info.id = message.identifier;
                info.data0 = (message.data_length_code > 0) ? message.data[0] : 0xFF; // 데이터가 없을 경우 대비
                xQueueSend(xQueue_Beep, &info, 0);
            }

            if (currentMode == MODE_ANALYZER || (currentMode == MODE_OPERATION && ENABLE_OP_MODE_LOGGING)) {
                printRawCanData(message);
            }

            if (message.data_length_code > 0) {
                bool stateChanged = false;

                if (xSemaphoreTake(xMutex_State, pdMS_TO_TICKS(2)) == pdTRUE) {
                    
                    if (message.identifier == ID_SEAT_DRIVER) {
                        parseFrontSeatStatus(message.data[0], g_currentCarState.fl_heat, g_currentCarState.fl_vent);
                    } else if (message.identifier == ID_SEAT_PASSENGER) {
                        parseFrontSeatStatus(message.data[0], g_currentCarState.fr_heat, g_currentCarState.fr_vent);
                    } else if (message.identifier == ID_SEAT_RL) {
                        parseRearSeatStatus(message.data[0], g_currentCarState.rl_heat);
                    } else if (message.identifier == ID_SEAT_RR) {
                        parseRearSeatStatus(message.data[0], g_currentCarState.rr_heat);
                    }

                    else if (message.identifier == ID_HVAC_AC) {
                        uint8_t val = message.data[6];
                        if (val == 0x05 || val == 0x09) g_currentCarState.ac = 1; else g_currentCarState.ac = 0;
                    }
                    else if (message.identifier == ID_HVAC_SYNC) {
                        uint8_t val = message.data[3];
                        if (val == 0x10) g_currentCarState.sync = 1; else g_currentCarState.sync = 0;
                    }
                    else if (message.identifier == ID_HVAC_MODE) {
                        uint8_t val = message.data[4];
                        if ((val >> 4) & 0x01) g_currentCarState.driver_only = 1; else g_currentCarState.driver_only = 0;
                        if (val & 0x01) g_currentCarState.heat = 1; else g_currentCarState.heat = 0;
                    }
                    else if (message.identifier == ID_STEER_HEAT) {
                        uint8_t val = message.data[3];
                        if (val == 0x01) g_currentCarState.steer_heat = 1; else g_currentCarState.steer_heat = 0;
                    }
                    
                    if (g_currentCarState.fl_heat != last_fl_h || g_currentCarState.fl_vent != last_fl_v ||
                        g_currentCarState.fr_heat != last_fr_h || g_currentCarState.fr_vent != last_fr_v ||
                        g_currentCarState.rl_heat != last_rl_h || g_currentCarState.rr_heat != last_rr_h ||
                        g_currentCarState.ac != last_ac || g_currentCarState.sync != last_sync ||
                        g_currentCarState.driver_only != last_drv || g_currentCarState.heat != last_heat ||
                        g_currentCarState.steer_heat != last_steer) 
                    {
                        stateChanged = true;
                        
                        last_fl_h = g_currentCarState.fl_heat; last_fl_v = g_currentCarState.fl_vent;
                        last_fr_h = g_currentCarState.fr_heat; last_fr_v = g_currentCarState.fr_vent;
                        last_rl_h = g_currentCarState.rl_heat; last_rr_h = g_currentCarState.rr_heat;
                        last_ac = g_currentCarState.ac; last_sync = g_currentCarState.sync;
                        last_drv = g_currentCarState.driver_only; last_heat = g_currentCarState.heat;
                        last_steer = g_currentCarState.steer_heat;
                    }

                    xSemaphoreGive(xMutex_State);
                }

                if (stateChanged) {
                    reportStatusToMaster();
                }
            }
        }
    }
}

void hvacControlTask(void *parameter) {
    HvacData newCmd;
    HvacData currentStateCopy;

    // [정의] HVAC 제어 데이터
    const uint8_t DRV_ONLY_ON[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFF};
    const uint8_t DRV_ONLY_OFF[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 0xFF, 0xFF};
    const uint8_t HEAT_ON[8]      = {0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0xFF};
    const uint8_t HEAT_OFF[8]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xF3, 0xFF, 0xFF, 0xFF};
    const uint8_t AC_ON[8]        = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF, 0xFF};
    const uint8_t AC_OFF[8]       = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0xFF, 0xFF};
    const uint8_t SYNC_ON[8]      = {0xFF, 0xFF, 0xFF, 0xFB, 0xFF, 0xFF, 0xFF, 0xFF};
    const uint8_t SYNC_OFF[8]     = {0xFF, 0xFF, 0xFF, 0xF7, 0xFF, 0xFF, 0xFF, 0xFF};

    // [정의] 종료 패킷들 (원본 데이터 확인 완료)
    const uint8_t SEAT_TERM[8] = {0x00, 0x00, 0xF3, 0x03, 0xFF, 0xFF, 0x00, 0x00};
    const uint8_t HVAC_TERM[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    while (1) {
        if (xQueueReceive(xQueue_Hvac, &newCmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            if (xSemaphoreTake(xMutex_State, pdMS_TO_TICKS(10)) == pdTRUE) {
                memcpy(&currentStateCopy, &g_currentCarState, sizeof(HvacData));
                xSemaphoreGive(xMutex_State);
            } else {
                continue; 
            }

            // 시트 제어 (Interleaved 적용)
            if (newCmd.fl_heat != currentStateCopy.fl_heat || newCmd.fl_vent != currentStateCopy.fl_vent) {
                uint8_t drvNibble = getFrontNibble(newCmd.fl_heat, newCmd.fl_vent);
                uint8_t byte4 = (drvNibble << 4) | 0x0F;
                uint8_t frameData[8] = {0x00, 0x00, 0xF3, 0x03, byte4, 0xFF, 0x00, 0x00};
                injectInterleavedBurst(ID_SEAT_CMD, frameData, SEAT_TERM);
            }
            if (newCmd.fr_heat != currentStateCopy.fr_heat || newCmd.fr_vent != currentStateCopy.fr_vent) {
                uint8_t psgNibble = getFrontNibble(newCmd.fr_heat, newCmd.fr_vent);
                uint8_t byte4 = 0xF0 | (psgNibble & 0x0F);
                uint8_t frameData[8] = {0x00, 0x00, 0xF3, 0x03, byte4, 0xFF, 0x00, 0x00};
                injectInterleavedBurst(ID_SEAT_CMD, frameData, SEAT_TERM);
            }
            if (newCmd.rl_heat != currentStateCopy.rl_heat) {
                uint8_t rlNibble = getRearNibble(newCmd.rl_heat);
                uint8_t byte5 = (rlNibble << 4) | 0x0F;
                uint8_t frameData[8] = {0x00, 0x00, 0xF3, 0x03, 0xFF, byte5, 0x00, 0x00};
                injectInterleavedBurst(ID_SEAT_CMD, frameData, SEAT_TERM);
            }
            if (newCmd.rr_heat != currentStateCopy.rr_heat) {
                uint8_t rrNibble = getRearNibble(newCmd.rr_heat);
                uint8_t byte5 = 0xF0 | (rrNibble & 0x0F);
                uint8_t frameData[8] = {0x00, 0x00, 0xF3, 0x03, 0xFF, byte5, 0x00, 0x00};
                injectInterleavedBurst(ID_SEAT_CMD, frameData, SEAT_TERM);
            }

            // HVAC 제어 (Interleaved 적용)
            if (newCmd.driver_only != currentStateCopy.driver_only) {
                if (newCmd.driver_only) injectInterleavedBurst(ID_HVAC_CMD_1, DRV_ONLY_ON, HVAC_TERM);
                else injectInterleavedBurst(ID_HVAC_CMD_1, DRV_ONLY_OFF, HVAC_TERM);
            }
            if (newCmd.heat != currentStateCopy.heat) {
                if (newCmd.heat) injectInterleavedBurst(ID_HVAC_CMD_1, HEAT_ON, HVAC_TERM);
                else injectInterleavedBurst(ID_HVAC_CMD_1, HEAT_OFF, HVAC_TERM);
            }
            if (newCmd.ac != currentStateCopy.ac) {
                if (newCmd.ac) injectInterleavedBurst(ID_HVAC_CMD_2, AC_ON, HVAC_TERM);
                else injectInterleavedBurst(ID_HVAC_CMD_2, AC_OFF, HVAC_TERM);
            }
            if (newCmd.sync != currentStateCopy.sync) {
                if (newCmd.sync) injectInterleavedBurst(ID_HVAC_CMD_3, SYNC_ON, HVAC_TERM);
                else injectInterleavedBurst(ID_HVAC_CMD_3, SYNC_OFF, HVAC_TERM);
            }

            // 핸들 열선
            // [복구] 대기 로직 없이 플래그 설정 후 즉시 비프음 전송
            // (CAN Task의 트리거와 거의 동시에 실행됨 - Interleaving 효과)
            if (newCmd.steer_heat != currentStateCopy.steer_heat) {
                g_waitSteerInjection = true; 
                
                if (g_beepEnabled) injectBeepBurstOnly();
            }
        }
    }
}

// ===============================================================
// 7. Setup & Loop
// ===============================================================
void setup() {
    Serial.begin(1000000);
    Serial1.begin(S3_BAUDRATE, SERIAL_8N1, PIN_S3_RX, PIN_S3_TX);

    xQueue_Hvac = xQueueCreate(5, sizeof(HvacData));
    xQueue_Beep = xQueueCreate(10, sizeof(BeepInfo)); // [수정] 큐 데이터 타입 변경 반영

    xMutex_State = xSemaphoreCreateMutex(); 

    // [NVS 로드 이동] 태스크 시작 전 가장 먼저 상태 로드
    // g_beepEnabled가 false로 저장되어 있었다면 여기서 즉시 false로 설정됨
    prefs.begin("car_config", false);
    g_beepEnabled = prefs.getBool("beep", true);
    Serial.printf("[SETUP] Beep Config Loaded: %s\n", g_beepEnabled ? "TRUE" : "FALSE");

    // 초기화: 모두 0으로 설정
    memset(&g_currentCarState, 0, sizeof(FeedbackData));

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_CAN_TX, (gpio_num_t)PIN_CAN_RX, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 20;

    twai_timing_config_t t_config;
    switch (CAN_BITRATE) {
        case 500000: t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
        default: t_config = TWAI_TIMING_CONFIG_500KBITS(); break;
    }

    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        twai_start();
    }

    // 태스크 생성
    xTaskCreate(s3LinkTask, "S3_LINK", 2048, NULL, 5, NULL);
    xTaskCreate(canTask, "CAN_OPS", 4096, NULL, 15, NULL);
    xTaskCreate(hvacControlTask, "HVAC", 4096, NULL, 5, NULL);
    xTaskCreate(beepMonitorTask, "BEEP", 2048, NULL, 1, NULL);
}

void loop() {
    vTaskDelete(NULL);
}