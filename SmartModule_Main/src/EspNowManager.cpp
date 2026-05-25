/**
 * [EspNowManager.cpp]
 * ESP-NOW 통신, 페어링 로직, FastLED 제어 구현
 * - 수정: sendHvacStatusForPairing에서 확장된 FeedbackData 사용 (0으로 초기화되어 전송됨)
 */

#include "EspNowManager.h"

extern void beep(int times);

// ===============================================================
// 전역 변수 정의
// ===============================================================
volatile bool g_wifiInitialized = false;
uint8_t g_remotePeerMac[6] = {0,};
bool g_isPeerRegistered = false;

volatile bool g_needInitialSync = false;
volatile bool g_hvacCommandReceived = false;
HvacData g_receivedHvacData;

volatile bool g_sendCbFired = false;
volatile esp_now_send_status_t g_lastSendStatus = ESP_NOW_SEND_FAIL;

volatile PairingState g_pairingState = PAIR_IDLE;
CRGB leds[NUM_LEDS];
Preferences preferences;
TaskHandle_t pairingTaskHandle = NULL;

uint8_t tempPeerMac[6];
uint8_t tempChannel = 1;
volatile bool g_pairingAckReceived = false;
uint8_t g_originalChannel = 1; 

// ===============================================================
// 내부 함수
// ===============================================================
bool registerPeer(const uint8_t *mac_addr, uint8_t channel) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, mac_addr, 6);
    peerInfo.channel = channel; 
    peerInfo.encrypt = false;
    
    if (esp_now_is_peer_exist(mac_addr)) {
        esp_now_del_peer(mac_addr);
    }
    
    return (esp_now_add_peer(&peerInfo) == ESP_OK);
}

// 0xA2 패킷 전송 (페어링 응답용)
// [참고] 확장된 구조체(FeedbackData)를 사용하지만 memset으로 0 초기화하므로
// 시간 정보는 0으로 채워져서 전송됩니다. (문제 없음)
void sendHvacStatusForPairing() {
    FeedbackData fbData;
    memset(&fbData, 0, sizeof(FeedbackData)); 
    fbData.packet_type = 0xA2; 
    
    // 현재 채널과 다를 경우에만 채널 변경
    if (WiFi.channel() != tempChannel) {
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(tempChannel, WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);
    }
    
    esp_err_t result = esp_now_send(tempPeerMac, (uint8_t*)&fbData, sizeof(FeedbackData));
    if (result != ESP_OK) {
        Serial.printf("[Pairing] Send Ack Fail: %s\n", esp_err_to_name(result));
    }
}

// ===============================================================
// [Task] 페어링 매니저 태스크
// ===============================================================
void pairingManagerTask(void *pvParameters) {
    const int HOPPING_INTERVAL = 600;
    const int LED_BLINK_INTERVAL = 1000;
    unsigned long lastHopTime = 0;
    unsigned long lastLedTime = 0;
    bool ledState = false;
    uint8_t currentScanChannel = 1;

    while (1) {
        if (g_pairingState == PAIR_IDLE) {
            vTaskDelay(pdMS_TO_TICKS(500)); 
            continue;
        }

        if (g_pairingState == PAIR_SCANNING) {
            unsigned long now = millis();

            if (now - lastLedTime >= LED_BLINK_INTERVAL) {
                lastLedTime = now;
                ledState = !ledState;
                leds[0] = ledState ? CRGB::White : CRGB::Black; 
                FastLED.show();
            }

            if (now - lastHopTime >= HOPPING_INTERVAL) {
                lastHopTime = now;
                currentScanChannel++;
                if (currentScanChannel > 13) currentScanChannel = 1;
                
                esp_wifi_set_promiscuous(true);
                esp_wifi_set_channel(currentScanChannel, WIFI_SECOND_CHAN_NONE);
                esp_wifi_set_promiscuous(false);
            }
        }
        
        else if (g_pairingState == PAIR_FOUND) {
            static unsigned long waitStart = 0;
            if (waitStart == 0) waitStart = millis();

            if (g_pairingAckReceived) {
                g_pairingState = PAIR_SUCCESS;
                waitStart = 0;
            } else if (millis() - waitStart > 1000) {
                Serial.println("[Pairing] No ACK, Retry Scanning...");
                g_pairingState = PAIR_SCANNING;
                waitStart = 0;
            }
        }

        else if (g_pairingState == PAIR_SUCCESS) {
            Serial.println("[Pairing] SUCCESS! Saving info...");

            preferences.begin("car_sys", false);
            preferences.putBytes("peer_mac", tempPeerMac, 6);
            preferences.putUChar("wifi_ch", tempChannel); 
            preferences.end();

            memcpy(g_remotePeerMac, tempPeerMac, 6);
            g_isPeerRegistered = true;
            g_needInitialSync = true; 

            for (int i = 0; i < 3; i++) {
                leds[0] = CRGB::Blue; FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(300));
                leds[0] = CRGB::Black; FastLED.show();
                vTaskDelay(pdMS_TO_TICKS(300));
            }
            
            // 페어링 완료 후 채널 고정
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_channel(tempChannel, WIFI_SECOND_CHAN_NONE);
            esp_wifi_set_promiscuous(false);
            
            Serial.printf("[Pairing] Exit Mode. Channel set to %d.\n", tempChannel);
            g_pairingState = PAIR_IDLE;
        }

        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ===============================================================
// ESP-NOW 콜백 함수들
// ===============================================================

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (g_pairingState == PAIR_FOUND) {
        if (status == ESP_NOW_SEND_SUCCESS) {
            g_pairingAckReceived = true; 
        }
    } else {
        g_lastSendStatus = status;
        g_sendCbFired = true;
    }
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void IRAM_ATTR OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataPtr, int len) {
    const uint8_t *mac = info->src_addr;
#else
void IRAM_ATTR OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len) {
#endif
    // 1. [페어링 모드]
    if (g_pairingState == PAIR_SCANNING) {
        if (len == sizeof(PairingData) && incomingDataPtr[0] == 0xB1) {
            PairingData pData;
            memcpy(&pData, incomingDataPtr, sizeof(PairingData));
            
            memcpy(tempPeerMac, pData.mac_addr, 6);
            tempChannel = WiFi.channel(); // 현재 채널 저장
            
            // 피어를 현재 채널(tempChannel)에 등록
            registerPeer(tempPeerMac, tempChannel);
            
            g_pairingAckReceived = false;
            g_pairingState = PAIR_FOUND;
            
            // 응답 전송 (이때 tempChannel을 유지해야 함)
            sendHvacStatusForPairing();
        }
        return;
    }

    // 2. [일반 모드]
    if (g_isPeerRegistered && memcmp(g_remotePeerMac, mac, 6) == 0) {
        // (A) 제어 명령 (0xA1)
        if (len == sizeof(HvacData) && incomingDataPtr[0] == 0xA1) {
            memcpy(&g_receivedHvacData, incomingDataPtr, sizeof(HvacData));
            g_hvacCommandReceived = true; 
        }
        // (B) 하트비트/연결확인 (0xB1)
        else if (len == sizeof(PairingData) && incomingDataPtr[0] == 0xB1) {
            g_needInitialSync = true; 
        }
    }
}

// ===============================================================
// 외부 인터페이스 함수
// ===============================================================

void startPairingMode() {
    if (g_pairingState == PAIR_SCANNING || g_pairingState == PAIR_FOUND) {
        Serial.println(">>> FORCE EXIT PAIRING MODE <<<");
        g_pairingState = PAIR_IDLE;
        
        if (g_originalChannel > 0) {
            esp_wifi_set_promiscuous(true);
            esp_wifi_set_channel(g_originalChannel, WIFI_SECOND_CHAN_NONE);
            esp_wifi_set_promiscuous(false);
        }

        leds[0] = CRGB::Black;
        FastLED.show();
        beep(2); 
        return;
    }

    if (g_pairingState == PAIR_IDLE) {
        if (g_isPeerRegistered) {
            if (esp_now_is_peer_exist(g_remotePeerMac)) {
                esp_now_del_peer(g_remotePeerMac);
            }
            g_isPeerRegistered = false;
            memset(g_remotePeerMac, 0, 6);
            Serial.println("[INFO] Cleared previous peer.");
        }

        g_originalChannel = WiFi.channel(); 
        Serial.println(">>> ENTER PAIRING MODE <<<");
        beep(4); 
        g_pairingState = PAIR_SCANNING;
    }
}

void setupEspNowWithLed() {
    FastLED.addLeds<WS2812, PIN_RGB_LED, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    leds[0] = CRGB::Black;
    FastLED.show();

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE); 
    
    preferences.begin("car_sys", true);
    uint8_t savedCh = preferences.getUChar("wifi_ch", 1); 
    size_t len = preferences.getBytes("peer_mac", g_remotePeerMac, 6);
    preferences.end();

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(savedCh, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERR] ESP-NOW Init Fail");
        leds[0] = CRGB::Red; FastLED.show();
        beep(5);
    } else {
        Serial.printf("[OK] ESP-NOW Ready (Ch: %d)\n", savedCh);
        if (len == 6) {
            registerPeer(g_remotePeerMac, savedCh);
            g_isPeerRegistered = true;
            Serial.println("Restored Peer from NVS");
        }
        esp_now_register_recv_cb(OnDataRecv);
        esp_now_register_send_cb(OnDataSent);
        
        leds[0] = CRGB::Green; FastLED.show();
        delay(500);
        leds[0] = CRGB::Black; FastLED.show();
    }
    g_wifiInitialized = true;

    xTaskCreate(pairingManagerTask, "PairingTask", 4096, NULL, 1, &pairingTaskHandle);
}