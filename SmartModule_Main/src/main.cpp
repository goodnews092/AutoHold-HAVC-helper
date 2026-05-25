/**
 * [프로젝트: ESP32-S3 SuperMini - 듀얼 CAN 오토홀드 + HVAC Gateway 통합]
 * - 역할: 메인 컨트롤러 (S3) - Receiver & Sender (Bidirectional)
 * - 모드: WIFI_STA (최적화 버전)
 * - 수정 사항:
 * 1. [2026-02-02] CAN ID 0x413(오토라이트), 0x4F0(시간/날짜) 파싱 로직 추가
 * 2. [수정] 0x4F0 데이터 매핑 오류 수정 (Byte 4:월, Byte 5:년)
 * - 증상: 년도가 72(0x48, Feb encoded)로 표시되는 문제 해결
 * 3. 분(Minute) 변경 시 memcmp 차이 발생 -> 즉시 전송 트리거됨
 */

#include <Arduino.h>
#include "driver/twai.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>      
#include <Preferences.h>
#include "EspNowManager.h" 

// ===============================================================
// 1. 하드웨어 핀 맵핑 (ESP32-S3 SuperMini)
// ===============================================================
#define PIN_TX        6   // CAN TX
#define PIN_RX        7   // CAN RX
#define PIN_RELAY     4   // 릴레이
#define PIN_BUZZER    5   // 부저
#define PIN_BUTTON    8   // 버튼
#define PIN_C3_RX     2   // C3 UART RX
#define PIN_C3_TX     1   // C3 UART TX

// ===============================================================
// 2. 시스템 설정 및 상수
// ===============================================================
#define TARGET_ID     0x1CF 
#define STATUS_ID     0x1E1 
#define BRAKE_ID      0x418 
#define CAN_BITRATE   500000 
#define C3_BAUDRATE   115200 

#define BOOT_DELAY_MS 5000 
#define INJECTION_DURATION_MS 1000 
#define LOGIC_STARTUP_DELAY_MS 3000 
#define CONST_SPD_15  15 
#define CONST_SPD_50  50 

#define WDT_TIMEOUT_SECONDS 10      
#define NVS_WRITE_INTERVAL_MS 5000  
#define DATA_TIMEOUT_MS 1000        

// ===============================================================
// 3. 전역 변수 및 데이터 구조
// ===============================================================
SemaphoreHandle_t xMutex_SharedData; 
Preferences mainPrefs; 

volatile bool g_canHwInitSuccess = false; 
volatile bool g_validDataReceived = false; 
volatile char g_lastAckChar = 0; 

// [참고] ESP-NOW 관련 변수는 EspNowManager.h에 extern으로 선언됨

struct SharedData {
  uint8_t gearState;      
  uint8_t ahStatus;       
  long rawSpeedVal;       
  bool btnCancelPressed;  
  bool isCruiseActive;    
  bool isBrakePressed;    
  unsigned long lastUpdateTime; 
  
  // 시트 열선/통풍 상태 (0-3)
  uint8_t fl_heat; uint8_t fl_vent;
  uint8_t fr_heat; uint8_t fr_vent;
  uint8_t rl_heat; uint8_t rr_heat;

  // HVAC 상태 필드 (0:Off, 1:On)
  uint8_t steer_heat;
  uint8_t driver_only;
  uint8_t ac;
  uint8_t heat;
  uint8_t sync;

  // [신규] 오토라이트 및 시간 정보
  uint8_t auto_light;
  uint8_t year;
  uint8_t month;
  uint8_t day;
  uint8_t weekday;
  uint8_t hour;
  uint8_t minute;
} sharedData;

enum Mode { MODE_ANALYZER, MODE_OPERATION };
volatile Mode currentMode = MODE_OPERATION; 

volatile bool g_needBootInjection = false;
volatile bool g_isInjecting = false;       
volatile unsigned long g_injectionStartTime = 0;

unsigned long lastNvsWriteTime = 0;
uint8_t pendingNvsValue = 0xFF;
bool isNvsWritePending = false;
uint8_t last1E1Byte5 = 0xFF; 

const uint8_t CRC_LUT[16] = {
  0xF6, 0x4F, 0x99, 0x20, 0x28, 0x91, 0x47, 0xFE, 
  0x57, 0xEE, 0x38, 0x81, 0x89, 0x30, 0xE6, 0xB5  
};

// 함수 원형
void beep(int times); 
void checkAndRecoverCAN();
void printRawCanData(twai_message_t message);
uint8_t calcWeekday(uint16_t y, uint8_t m, uint8_t d); // 요일 계산 함수

// ===============================================================
// 4. 유틸리티 함수
// ===============================================================

static inline uint32_t calc_burst_delay_us(uint8_t dlc) {
    uint32_t bits = 55 + (dlc * 8);
    return (bits * 2) + 20; 
}

void IRAM_ATTR fastInjectionBurst(uint8_t detectedCounter) {
    twai_message_t tx_msg;
    tx_msg.identifier = TARGET_ID;
    tx_msg.extd = 0;
    tx_msg.data_length_code = 8;
    tx_msg.rtr = 0;
    
    tx_msg.data[2] = 0x80; tx_msg.data[3] = 0x20; 
    tx_msg.data[4] = 0x00; tx_msg.data[5] = 0x00; tx_msg.data[6] = 0x00; tx_msg.data[7] = 0x00;

    uint8_t currentIdx = (detectedCounter & 0x0F); 
    uint32_t delay_us = calc_burst_delay_us(8);

    for (int i = 1; i <= 4; i++) { 
        uint8_t nextCnt = (currentIdx + i) % 15; 
        tx_msg.data[1] = (nextCnt << 4);     
        tx_msg.data[0] = CRC_LUT[nextCnt];  
        twai_transmit(&tx_msg, 0);
        delayMicroseconds(delay_us);     
    }
}

// 요일 계산 함수 (0=일요일, 6=토요일)
// Sakamoto's methods algorithm
uint8_t calcWeekday(uint16_t y, uint8_t m, uint8_t d) {
    // 2000년대라고 가정 (y는 2자리 숫자, 예: 26 -> 2026)
    int yearFull = 2000 + y;
    static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    yearFull -= m < 3;
    return (yearFull + yearFull/4 - yearFull/100 + yearFull/400 + t[m-1] + d) % 7;
}

void beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100)); 
    digitalWrite(PIN_BUZZER, LOW);
    vTaskDelay(pdMS_TO_TICKS(100)); 
  }
}

void checkAndRecoverCAN() {
    twai_status_info_t status_info;
    if (twai_get_status_info(&status_info) == ESP_OK) {
        if (status_info.state == TWAI_STATE_BUS_OFF) twai_initiate_recovery(); 
        if (status_info.state == TWAI_STATE_STOPPED) twai_start();
    }
}

void printRawCanData(twai_message_t message) {
    if (message.extd) Serial.print("T"); else Serial.print("t");
    char idBuffer[9];
    sprintf(idBuffer, message.extd ? "%08X" : "%03X", (unsigned int)message.identifier);
    Serial.print(idBuffer);
    Serial.print(message.data_length_code);
    for (int i = 0; i < message.data_length_code; i++) {
        char byteBuffer[3];
        sprintf(byteBuffer, "%02X", message.data[i]);
        Serial.print(byteBuffer);
    }
    Serial.print("\r"); 
}

// ===============================================================
// 5. 태스크 (Tasks)
// ===============================================================

// C3 UART 수신 태스크 (Legacy 구조체 사용)
void c3LinkTask(void *parameter) {
  uint8_t rxBuf[sizeof(FeedbackData_Legacy)]; 

  while (1) {
    if (Serial1.available()) {
        uint8_t header = Serial1.peek();

        // 1. C3에서 온 상태 보고 데이터 (0xA2)
        if (header == 0xA2) {
            if (Serial1.available() >= sizeof(FeedbackData_Legacy)) {
                Serial1.readBytes(rxBuf, sizeof(FeedbackData_Legacy));
                FeedbackData_Legacy* pData = (FeedbackData_Legacy*)rxBuf;

                if (xSemaphoreTake(xMutex_SharedData, pdMS_TO_TICKS(5)) == pdTRUE) {
                    sharedData.steer_heat = pData->steer_heat;
                    sharedData.ac = pData->ac;
                    sharedData.heat = pData->heat;
                    sharedData.sync = pData->sync;
                    sharedData.driver_only = pData->driver_only;
                    
                    sharedData.lastUpdateTime = millis(); 
                    xSemaphoreGive(xMutex_SharedData);
                }
            }
        }
        // 2. 문자열 처리 (버튼 ACK 등)
        else {
            String line = Serial1.readStringUntil('\n'); 
            line.trim();
            if (line.startsWith("ACK:")) {
                g_lastAckChar = line.charAt(4); 
            }
        }
    }
    vTaskDelay(pdMS_TO_TICKS(10)); 
  }
}

void canReceiveTask(void *parameter) {
  esp_task_wdt_add(NULL);
  twai_message_t message;

  while (1) {
    esp_task_wdt_reset();
    if (!g_canHwInitSuccess) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
    }
    checkAndRecoverCAN();

    if (isNvsWritePending && (millis() - lastNvsWriteTime > NVS_WRITE_INTERVAL_MS)) {
        mainPrefs.putUChar("val_1e1", pendingNvsValue);
        lastNvsWriteTime = millis();
        isNvsWritePending = false;
    }

    if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
      if (!g_validDataReceived) g_validDataReceived = true; 

      if (currentMode == MODE_OPERATION) {
          unsigned long now = millis();
          if (g_needBootInjection && message.identifier == TARGET_ID) {
              if (now >= BOOT_DELAY_MS) {
                  if (!g_isInjecting) { 
                      g_isInjecting = true; 
                      g_injectionStartTime = now; 
                  }
                  if (g_isInjecting) {
                      if (now - g_injectionStartTime < INJECTION_DURATION_MS) {
                          uint8_t rxCounter = (message.data[1] >> 4) & 0x0F;
                          fastInjectionBurst(rxCounter);
                          continue; 
                      } else { 
                          g_isInjecting = false; 
                          g_needBootInjection = false; 
                      }
                  }
              }
          }

          if (message.identifier == STATUS_ID) { 
              uint8_t currentByte5 = message.data[5];
              if (currentByte5 != last1E1Byte5) {
                  last1E1Byte5 = currentByte5;
                  pendingNvsValue = currentByte5;
                  isNvsWritePending = true;
              }
          }

          uint8_t s_heat = 0, s_vent = 0;
          bool isSeatMsg = false;
          
          if (message.identifier == 0x496 || message.identifier == 0x475) { 
             uint8_t val = message.data[0];
             if (val == 0x46) s_heat = 3;
             else if (val == 0x3E) s_heat = 2;
             else if (val == 0x36) s_heat = 1;
             else if (val == 0x2E) s_vent = 3;
             else if (val == 0x26) s_vent = 2;
             else if (val == 0x1E) s_vent = 1;
             isSeatMsg = true;
          } else if (message.identifier == 0x438 || message.identifier == 0x453) { 
             uint8_t val = message.data[0];
             if (val == 0x41) s_heat = 2;
             else if (val == 0x31) s_heat = 1;
             isSeatMsg = true;
          }

          if (xSemaphoreTake(xMutex_SharedData, pdMS_TO_TICKS(5)) == pdTRUE) { 
            if (!message.extd) {
               bool dataUpdated = false;
               
               if (isSeatMsg) {
                   if (message.identifier == 0x496) { sharedData.fl_heat = s_heat; sharedData.fl_vent = s_vent; }
                   else if (message.identifier == 0x475) { sharedData.fr_heat = s_heat; sharedData.fr_vent = s_vent; }
                   else if (message.identifier == 0x438) { sharedData.rl_heat = s_heat; }
                   else if (message.identifier == 0x453) { sharedData.rr_heat = s_heat; }
                   dataUpdated = true;
               }

               switch (message.identifier) {
                 // 1. 오토라이트 (0x413)
                 case 0x413: {
                     bool lightOn = (message.data[3] & 0x01);
                     sharedData.auto_light = lightOn ? 1 : 0;
                     dataUpdated = true; 
                     break;
                 }

                 // 2. 시간/날짜 (0x4F0)
                 // [수정] Byte Mapping 교체
                 // Byte 4(index 4): Month (Encoded 0x40 + M*4) -> 값 72는 2월(0x48)
                 // Byte 5(index 5): Year (Offset) -> 값 26
                 // Byte 6(index 6): Day -> 값 03
                 case 0x4F0: {
                     sharedData.hour = message.data[1];
                     sharedData.minute = message.data[2];
                     
                     // [수정] 월 파싱 (index 4)
                     if (message.data[4] >= 0x40) {
                         sharedData.month = (message.data[4] - 0x40) / 4;
                     } else {
                         sharedData.month = 1; 
                     }
                     
                     // [수정] 년 파싱 (index 5)
                     sharedData.year = message.data[5];
                     
                     // 일 파싱 (index 6)
                     sharedData.day = message.data[6];
                     
                     // 요일 재계산 (자동)
                     sharedData.weekday = calcWeekday(sharedData.year, sharedData.month, sharedData.day);
                     
                     dataUpdated = true;
                     break;
                 }

                 case 0x064: sharedData.ahStatus = message.data[3]; dataUpdated = true; break;
                 case 0x039: sharedData.gearState = message.data[0]; dataUpdated = true; break;
                 case 0x1AC: sharedData.rawSpeedVal = message.data[0]; dataUpdated = true; break;
                 case 0x1A2: sharedData.isCruiseActive = (message.data[0] != 0x00); dataUpdated = true; break;
                 case BRAKE_ID: sharedData.isBrakePressed = (message.data[5] & 0x40) ? true : false; dataUpdated = true; break;
                 case TARGET_ID: 
                   bool isPressedNow = (message.data[2] == 0x04);
                   if (isPressedNow && !sharedData.btnCancelPressed) {
                       sharedData.btnCancelPressed = true; 
                   }
                   dataUpdated = true; 
                   break;
               }
               
               if (dataUpdated) {
                   sharedData.lastUpdateTime = millis();
               }
            }
            xSemaphoreGive(xMutex_SharedData);
          }
      }
      else if (currentMode == MODE_ANALYZER) {
          printRawCanData(message); 
      }
    }
  }
}

void logicTask(void *parameter) {
  esp_task_wdt_add(NULL);
  const TickType_t loopInterval = pdMS_TO_TICKS(50); 
  
  static FeedbackData lastSentFeedback = {}; 
  static bool firstRun = true;
  static bool lastCancelBtnState = false;

  uint8_t localGear = 0xFF, localAh = 0xFF;
  int currentSpeed = 0;
  bool localCancelBtn = false, localCruiseActive = false, localBrakePressed = false;
  unsigned long lastDataTime = 0; 
  uint8_t filteredGear = 0; 
  int neutralCounter = 0;
  bool waitingForFeedback = false;
  unsigned long actionTime = 0;
  bool flagReverseDisabled = false, flagTempPause = false, flagManualOff = false;     
  uint8_t prevAhStatus = 0xFF;
  bool isFirstBootLoop = true; 
  
  bool lastBtnState = HIGH; 
  unsigned long btnPressStartTime = 0;
  bool btnLongPressHandled = false;

  while (1) {
    esp_task_wdt_reset();

    // 1. 버튼 로직 (Short: 모드변경 / Long 3s: 페어링)
    bool currentBtnState = digitalRead(PIN_BUTTON);
    
    if (currentBtnState == LOW && lastBtnState == HIGH) {
        btnPressStartTime = millis();
        btnLongPressHandled = false;
    } 
    else if (currentBtnState == LOW && lastBtnState == LOW) {
        if (!btnLongPressHandled && (millis() - btnPressStartTime > 3000)) {
            startPairingMode(); 
            btnLongPressHandled = true; 
        }
    }
    else if (currentBtnState == HIGH && lastBtnState == LOW) {
        if (!btnLongPressHandled && (millis() - btnPressStartTime > 50)) {
            Mode targetMode = (currentMode == MODE_OPERATION) ? MODE_ANALYZER : MODE_OPERATION;
            char targetCmd = (targetMode == MODE_ANALYZER) ? 'A' : 'O';
            
            g_lastAckChar = 0;
            Serial1.println(targetCmd); 

            unsigned long waitStart = millis();
            bool ackReceived = false;
            while(millis() - waitStart < 200) {
                if (g_lastAckChar == targetCmd) { 
                    ackReceived = true;
                    break;
                }
                vTaskDelay(1); 
            }

            if (ackReceived) {
                currentMode = targetMode; 
                beep(targetMode == MODE_ANALYZER ? 1 : 2); 
            } else {
                beep(3); 
            }
        }
    }
    lastBtnState = currentBtnState;

    if (!g_validDataReceived) { vTaskDelay(loopInterval); continue; }
    if (millis() < LOGIC_STARTUP_DELAY_MS) { vTaskDelay(loopInterval); continue; }
    if (currentMode == MODE_ANALYZER) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }

    if (xSemaphoreTake(xMutex_SharedData, pdMS_TO_TICKS(20)) == pdTRUE) {
      localGear = sharedData.gearState;
      localAh = sharedData.ahStatus;
      currentSpeed = (int)sharedData.rawSpeedVal;
      localCruiseActive = sharedData.isCruiseActive;
      localBrakePressed = sharedData.isBrakePressed; 
      lastDataTime = sharedData.lastUpdateTime; 
      
      bool currentCancelBtn = sharedData.btnCancelPressed;
      
      if (currentCancelBtn && !lastCancelBtnState) { 
          localCancelBtn = true;
      } else {
          localCancelBtn = false;
      }
      
      if(currentCancelBtn) {
          sharedData.btnCancelPressed = false; 
      }
      lastCancelBtnState = currentCancelBtn;

      xSemaphoreGive(xMutex_SharedData);
    } else {
      vTaskDelay(loopInterval); 
      continue;
    }
    
    // --- [피드백 전송 로직] ---
    if (g_isPeerRegistered && g_wifiInitialized && g_pairingState == PAIR_IDLE) {
        FeedbackData currentFeedback = {}; 
        currentFeedback.packet_type = 0xA2;
        
        if (g_needInitialSync) {
            static unsigned long lastRequestTime = 0;
            if (millis() - lastRequestTime > 2000) { 
                Serial1.println("S"); 
                lastRequestTime = millis();
            }
        }

        if (xSemaphoreTake(xMutex_SharedData, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentFeedback.fl_heat = sharedData.fl_heat;
            currentFeedback.fl_vent = sharedData.fl_vent;
            currentFeedback.fr_heat = sharedData.fr_heat;
            currentFeedback.fr_vent = sharedData.fr_vent;
            currentFeedback.rl_heat = sharedData.rl_heat;
            currentFeedback.rr_heat = sharedData.rr_heat;
            
            currentFeedback.steer_heat = sharedData.steer_heat; 
            currentFeedback.driver_only = sharedData.driver_only; 
            currentFeedback.ac = sharedData.ac;
            currentFeedback.heat = sharedData.heat;
            currentFeedback.sync = sharedData.sync;
            
            // [업데이트됨] CAN에서 읽은 시간 정보 반영
            currentFeedback.auto_light = sharedData.auto_light;
            currentFeedback.year = sharedData.year;
            currentFeedback.month = sharedData.month;
            currentFeedback.day = sharedData.day;
            currentFeedback.weekday = sharedData.weekday;
            currentFeedback.hour = sharedData.hour;
            currentFeedback.minute = sharedData.minute;

            xSemaphoreGive(xMutex_SharedData);
            
            bool isChanged = false;
            // [중요] memcmp는 구조체 전체 바이트를 비교합니다.
            // currentFeedback.minute 값이 1분 바뀌면 lastSentFeedback과 달라지므로
            // isChanged는 true가 되고, 아래에서 send 로직을 실행하게 됩니다.
            if (memcmp(&currentFeedback, &lastSentFeedback, sizeof(FeedbackData)) != 0) {
                isChanged = true;
            }

            if (g_needInitialSync || isChanged || firstRun) {
                g_sendCbFired = false; 
                esp_err_t result = esp_now_send(g_remotePeerMac, (uint8_t*)&currentFeedback, sizeof(FeedbackData));
                
                if (result == ESP_OK) {
                    unsigned long sendStart = millis();
                    while (!g_sendCbFired && (millis() - sendStart < 100)) {
                         vTaskDelay(1);
                    }
                    
                    if (g_sendCbFired && g_lastSendStatus == ESP_NOW_SEND_SUCCESS) {
                        memcpy(&lastSentFeedback, &currentFeedback, sizeof(FeedbackData));
                        firstRun = false;
                        if (g_needInitialSync) {
                            g_needInitialSync = false; 
                            Serial.println("[INFO] Init Sync Complete");
                        }
                    }
                }
            }
        }
    }
    
    if (millis() - lastDataTime > DATA_TIMEOUT_MS) {
        waitingForFeedback = false; 
        vTaskDelay(loopInterval); 
        continue;
    }

    // === 오토홀드 로직 (기존 유지) ===
    if (isFirstBootLoop) {
        if (localAh == 0x04) flagManualOff = true; 
        prevAhStatus = localAh;
        isFirstBootLoop = false;
    }

    if (localAh != prevAhStatus) {
        if (localAh == 0x04) { 
            if (!waitingForFeedback) { flagManualOff = true; flagTempPause = false; } 
        }
        else if (localAh == 0x20 || localAh == 0x30) { 
            if (!waitingForFeedback) { flagManualOff = false; flagTempPause = false; } 
        }
        prevAhStatus = localAh;
    }

    if (localGear == 0x06) {
      neutralCounter++; 
      if (neutralCounter >= 3) filteredGear = 0x06;       
    } else {
      filteredGear = localGear; 
      neutralCounter = 0;
    }

    if (waitingForFeedback) {
      if (millis() - actionTime > 1500) waitingForFeedback = false; 
      vTaskDelay(loopInterval); 
      continue;
    }

    bool relayTriggered = false;

    if (localCancelBtn && !localCruiseActive && currentSpeed < CONST_SPD_50) {
        if (!flagManualOff) {
            beep(1); 
            if (flagTempPause) {
                flagTempPause = false; 
                if (localAh == 0x04) relayTriggered = true; 
            } else {
                bool shouldTrigger = false;
                if (localAh == 0x20) { if (localBrakePressed) shouldTrigger = true; } 
                else if (localAh != 0x04) { shouldTrigger = true; }
                
                if (shouldTrigger) relayTriggered = true;
                flagTempPause = true; 
                flagReverseDisabled = false; 
            }
        }
    }

    if (flagTempPause && currentSpeed >= CONST_SPD_50) {
        flagTempPause = false; 
        if (localAh == 0x04) relayTriggered = true; 
    }

    if (filteredGear == 0x07) {
      flagReverseDisabled = true; 
      if (!flagTempPause && !flagManualOff) {
          if (!relayTriggered) {
              if (localAh == 0x30 || (localAh == 0x20 && localBrakePressed)) {
                  relayTriggered = true;
              }
          }
      }
    } 

    if (!relayTriggered && filteredGear == 0x05 && currentSpeed >= CONST_SPD_15 && flagReverseDisabled) {
      if (!flagManualOff && localAh == 0x04) { 
          relayTriggered = true; 
          flagTempPause = false; 
      }
      flagReverseDisabled = false;
    }

    if (!relayTriggered && filteredGear == 0x05 && currentSpeed == 0 && localAh == 0x04) {
      if (!flagTempPause && !flagManualOff && !flagReverseDisabled) {
          relayTriggered = true; 
      }
    }

    if (filteredGear == 0x00) {
        flagReverseDisabled = false; 
        flagTempPause = false; 
        if (!flagManualOff && !relayTriggered && localAh == 0x04) {
            relayTriggered = true;
        }
    }

    if (relayTriggered) {
      digitalWrite(PIN_RELAY, HIGH);
      vTaskDelay(pdMS_TO_TICKS(200)); 
      digitalWrite(PIN_RELAY, LOW);
      waitingForFeedback = true; 
      actionTime = millis();
    }
    vTaskDelay(loopInterval);
  }
}

// ===============================================================
// 6. Setup & Loop
// ===============================================================
void setup() {
  setCpuFrequencyMhz(160);
  Serial.begin(1000000);
  delay(100); 

  Serial.println("\n\n=== SYSTEM START (Optimized) ===");
  
  setupEspNowWithLed(); 
  WiFi.setTxPower(WIFI_POWER_11dBm);
  
  Serial1.begin(C3_BAUDRATE, SERIAL_8N1, PIN_C3_RX, PIN_C3_TX);

  pinMode(PIN_RELAY, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  digitalWrite(PIN_RELAY, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // SharedData 초기화
  sharedData.gearState = 0xFF;
  sharedData.ahStatus = 0xFF;
  sharedData.rawSpeedVal = 0;
  sharedData.fl_heat = 0; sharedData.fl_vent = 0;
  sharedData.fr_heat = 0; sharedData.fr_vent = 0;
  sharedData.rl_heat = 0; sharedData.rr_heat = 0;
  sharedData.steer_heat = 0; sharedData.driver_only = 0;
  sharedData.ac = 0; sharedData.heat = 0; sharedData.sync = 0;
  
  // 시간 정보 초기화
  sharedData.auto_light = 0;
  sharedData.year = 0; sharedData.month = 0; sharedData.day = 0;
  sharedData.weekday = 0; sharedData.hour = 0; sharedData.minute = 0;

  esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  xMutex_SharedData = xSemaphoreCreateMutex();
  sharedData.lastUpdateTime = millis(); 

  mainPrefs.begin("car_sys", false); 
  uint8_t saved1E1 = mainPrefs.getUChar("val_1e1", 0x00); 
  last1E1Byte5 = saved1E1; 
  g_needBootInjection = (saved1E1 == 0x80);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)PIN_TX, (gpio_num_t)PIN_RX, TWAI_MODE_NORMAL);
  g_config.rx_queue_len = 50; g_config.tx_queue_len = 20; 
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    if (twai_start() == ESP_OK) {
        g_canHwInitSuccess = true; 
    } else g_canHwInitSuccess = false;
  } else g_canHwInitSuccess = false;

  xTaskCreate(c3LinkTask, "C3_LINK", 4096, NULL, 5, NULL); 
  xTaskCreate(canReceiveTask, "CAN_RX", 4096, NULL, 15, NULL); 
  xTaskCreate(logicTask, "LOGIC", 4096, NULL, 5, NULL); 

  vTaskDelay(pdMS_TO_TICKS(500)); 
  Serial1.println("S"); 
}

void loop() {
  if (g_hvacCommandReceived) {
      g_hvacCommandReceived = false; 
      Serial1.write((uint8_t*)&g_receivedHvacData, sizeof(HvacData));
      // beep(1); 
  }

  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 10000) {
      lastStatusTime = millis();
      Serial.print("[INFO] MyMAC: ");
      Serial.print(WiFi.macAddress());
      Serial.print(" | Ch: ");
      Serial.print(WiFi.channel());
      
      Serial.print(" | Peer: ");
      if (g_isPeerRegistered) {
          char peerStr[18];
          snprintf(peerStr, sizeof(peerStr), "%02X:%02X:%02X:%02X:%02X:%02X",
                   g_remotePeerMac[0], g_remotePeerMac[1], g_remotePeerMac[2], 
                   g_remotePeerMac[3], g_remotePeerMac[4], g_remotePeerMac[5]);
          Serial.print(peerStr);
      } else {
          Serial.print("NONE");
      }

      if (g_pairingState == PAIR_SCANNING) Serial.print(" [PAIRING...]");
      else if (g_pairingState == PAIR_FOUND) Serial.print(" [FOUND]");
      else if (g_pairingState == PAIR_SUCCESS) Serial.print(" [DONE]");
      
      Serial.println();
  }

  vTaskDelay(pdMS_TO_TICKS(10)); 
}