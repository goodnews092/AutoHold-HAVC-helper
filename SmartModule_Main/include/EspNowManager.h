/**
 * [EspNowManager.h]
 * ESP-NOW 통신 및 페어링 매니저 헤더
 * - 수정: C3 호환성을 위한 Legacy 구조체 분리 및 확장 구조체 정의 (2026-02-02)
 */

#ifndef ESP_NOW_MANAGER_H
#define ESP_NOW_MANAGER_H

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <FastLED.h>
#include <Preferences.h>

// ===============================================================
// 설정 상수
// ===============================================================
#define ESPNOW_CHANNEL_DEFAULT 1
#define PIN_RGB_LED           48   // WS2812 LED Pin
#define NUM_LEDS              1
#define BRIGHTNESS            60

// ===============================================================
// 데이터 구조체
// ===============================================================

// HVAC 데이터 구조체 (수신용: Display -> S3) - 12 Bytes (0xA1)
struct __attribute__((packed)) HvacData {
    uint8_t packet_type;  // 0xA1
    uint8_t fl_heat; uint8_t fr_heat; uint8_t fl_vent; uint8_t fr_vent;
    uint8_t steer_heat;
    uint8_t rl_heat; uint8_t rr_heat;
    uint8_t driver_only; uint8_t ac; uint8_t heat; uint8_t sync;
};

// 페어링 데이터 구조체 (수신용) - 7 Bytes (0xB1)
struct __attribute__((packed)) PairingData {
    uint8_t packet_type;  // 0xB1
    uint8_t mac_addr[6];  // Sender MAC
};

// [C3 통신용] 기존 구조체 (변경 없음, UART 호환성 유지)
struct __attribute__((packed)) FeedbackData_Legacy {
    uint8_t packet_type;  // 0xA2
    uint8_t fl_heat; uint8_t fr_heat; uint8_t fl_vent; uint8_t fr_vent;
    uint8_t steer_heat;
    uint8_t rl_heat; uint8_t rr_heat;
    uint8_t driver_only; uint8_t ac; uint8_t heat; uint8_t sync;
};

// [디스플레이 전송용] 확장 구조체 (필드 추가됨)
struct __attribute__((packed)) FeedbackData {
    uint8_t packet_type;  // 0xA2
    
    // HVAC 상태 (Legacy와 동일 위치 매핑)
    uint8_t fl_heat; uint8_t fr_heat; uint8_t fl_vent; uint8_t fr_vent;
    uint8_t steer_heat;
    uint8_t rl_heat; uint8_t rr_heat;
    uint8_t driver_only; uint8_t ac; uint8_t heat; uint8_t sync;

    // [신규] 오토라이트 및 시간/날짜 정보 (디스플레이 전용)
    uint8_t auto_light;   // 0:주간, 1:야간
    uint8_t year;         // 년 (YY 형식, 예: 26)
    uint8_t month;        // 월 (1-12)
    uint8_t day;          // 일 (1-31)
    uint8_t weekday;      // 요일 (0:일, 1:월, ..., 6:토)
    uint8_t hour;         // 시 (0-23)
    uint8_t minute;       // 분 (0-59)
};

// ===============================================================
// 전역 변수 (extern)
// ===============================================================
extern volatile bool g_wifiInitialized;
extern uint8_t g_remotePeerMac[6];
extern bool g_isPeerRegistered;
extern volatile bool g_needInitialSync;
extern volatile bool g_hvacCommandReceived;
extern HvacData g_receivedHvacData;

// main.cpp에서 참조하는 전송 관련 변수
extern volatile bool g_sendCbFired;
extern volatile esp_now_send_status_t g_lastSendStatus;

// 페어링 상태 관리
enum PairingState {
    PAIR_IDLE,          // 평상시
    PAIR_SCANNING,      // 채널 호핑 중
    PAIR_FOUND,         // 0xB1 발견, 전송 시도 중
    PAIR_SUCCESS        // ACK 수신, 저장 및 종료 중
};
extern volatile PairingState g_pairingState;

// ===============================================================
// 함수 선언
// ===============================================================
void setupEspNowWithLed();       // 초기화 함수 (main.cpp setup에서 호출)
void startPairingMode();         // 페어링 모드 진입 요청
bool registerPeer(const uint8_t *mac_addr, uint8_t channel);

// 콜백 함수들
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void IRAM_ATTR OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataPtr, int len);
#else
void IRAM_ATTR OnDataRecv(const uint8_t * mac, const uint8_t *incomingDataPtr, int len);
#endif

#endif // ESP_NOW_MANAGER_H