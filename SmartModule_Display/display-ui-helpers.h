// display-ui-helpers.h
#pragma once
#include "esphome.h"
#include "lvgl.h"
#include <cstdio>

// ------------------------------------------------------------
// [송신] 0xA1: HVAC 제어 명령 (Display -> S3)
// ------------------------------------------------------------
struct __attribute__((packed)) HvacData {
    uint8_t packet_type;  // 0xA1
    
    uint8_t fl_heat;      // 운전석 열선 (0-3)
    uint8_t fr_heat;      // 조수석 열선 (0-3)
    uint8_t fl_vent;      // 운전석 통풍 (0-3)
    uint8_t fr_vent;      // 조수석 통풍 (0-3)
    
    uint8_t steer_heat;   // 핸들 열선 (0:Off, 1:On)
    
    uint8_t rl_heat;      // 뒷좌석 좌측 열선 (0-2)
    uint8_t rr_heat;      // 뒷좌석 우측 열선 (0-2)
    
    uint8_t driver_only;  // DRIVER ONLY 모드
    uint8_t ac;           // A/C 상태
    uint8_t heat;         // HEAT 상태
    uint8_t sync;         // SYNC 상태
};

// ------------------------------------------------------------
// [송신] 0xB1: 페어링 신호 (Display -> S3, Broadcast)
// ------------------------------------------------------------
struct __attribute__((packed)) PairingData {
    uint8_t packet_type;  // 0xB1
    uint8_t mac_addr[6];  // Sender MAC
};

// ------------------------------------------------------------
// [수신] 0xA2: 차량 상태 피드백 (S3 -> Display)
// [수정] year 필드를 uint8_t로 최적화 (YY 형식)
// ------------------------------------------------------------
struct __attribute__((packed)) FeedbackData {
    uint8_t packet_type;  // 0xA2
    
    // 기존 HVAC 상태
    uint8_t fl_heat;      
    uint8_t fr_heat;      
    uint8_t fl_vent;      
    uint8_t fr_vent;      
    uint8_t steer_heat;   
    uint8_t rl_heat;      
    uint8_t rr_heat;      
    uint8_t driver_only;  
    uint8_t ac;           
    uint8_t heat;         
    uint8_t sync;

    // 오토라이트 및 시간/날짜 정보
    uint8_t auto_light;   // 0:주간, 1:야간
    
    uint8_t year;         // [최적화] 년 (예: 26) - 1바이트
    uint8_t month;        // 월 (1-12)
    uint8_t day;          // 일 (1-31)
    uint8_t weekday;      // 요일 (0:일, 1:월, ..., 6:토)
    uint8_t hour;         // 시 (0-23)
    uint8_t minute;       // 분 (0-59)
};

// ------------------------------------------------------------
// UI Helper Functions
// ------------------------------------------------------------

static uint32_t last_user_interaction = 0;
static bool is_screensaver_active = false;
static lv_obj_t *screensaver_layer = nullptr;
static lv_obj_t *screensaver_clock_label = nullptr;
static lv_obj_t *screensaver_date_label = nullptr; 

// 요일 문자열 배열
static const char *WEEKDAY_STR[] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

/**
 * 인디케이터(LED) 초기 설정 함수
 */
void init_indicator(lv_obj_t *obj) {
    if (obj == nullptr) return;

    lv_obj_set_style_shadow_width(obj, 0, 0);
    lv_obj_set_style_shadow_opa(obj, 0, 0);
    lv_obj_set_style_border_width(obj, 0, 0);
    lv_obj_set_style_outline_width(obj, 0, 0);

    lv_color_t off_color = lv_color_hex(0x2A2A2A);
    lv_led_set_color(obj, off_color);
    lv_obj_set_style_bg_color(obj, off_color, 0);

    lv_led_off(obj);
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_ADV_HITTEST);
    lv_obj_invalidate(obj);
}

/**
 * 인디케이터(LED) 상태 및 색상 업데이트
 */
void update_indicator(lv_obj_t *obj, bool state, lv_color_t color) {
    if (obj == nullptr) return;

    if (state) {
        lv_led_set_color(obj, color);
        lv_obj_set_style_bg_color(obj, color, 0);
        lv_led_on(obj);
        lv_led_set_brightness(obj, 255);

        bool is_orange = (color.full == lv_color_hex(0xFF8000).full);
        if (is_orange) {
            lv_obj_set_style_shadow_width(obj, 14, 0);
            lv_obj_set_style_shadow_spread(obj, 2, 0);
            lv_obj_set_style_shadow_opa(obj, LV_OPA_90, 0);
        } else {
            lv_obj_set_style_shadow_width(obj, 8, 0);
            lv_obj_set_style_shadow_spread(obj, 1, 0);
            lv_obj_set_style_shadow_opa(obj, LV_OPA_60, 0);
        }
        lv_obj_set_style_shadow_color(obj, color, 0);   
    } else {
        lv_color_t off_color = lv_color_hex(0x2A2A2A);
        lv_led_set_color(obj, off_color);
        lv_obj_set_style_bg_color(obj, off_color, 0);
        lv_led_off(obj);
        lv_led_set_brightness(obj, 60);

        lv_obj_set_style_shadow_width(obj, 0, 0);
        lv_obj_set_style_shadow_opa(obj, 0, 0);
    }
    lv_obj_invalidate(obj);
}

void update_text_glow(lv_obj_t *label, bool state, lv_color_t color) {
    if (label == nullptr) return;
    lv_obj_set_style_text_color(label, state ? color : lv_color_hex(0xFFFFFF), 0);
    lv_obj_invalidate(label);
}

// ------------------------------------------------------------
// 스크린세이버(시계/날짜) 관리 로직
// ------------------------------------------------------------

void register_user_interaction() {
    last_user_interaction = millis();
    
    if (is_screensaver_active && screensaver_layer != nullptr) {
        lv_obj_add_flag(screensaver_layer, LV_OBJ_FLAG_HIDDEN);
        is_screensaver_active = false;
        lv_obj_invalidate(lv_scr_act());
    }
}

static void screensaver_click_cb(lv_event_t * e) {
    register_user_interaction();
}

/**
 * 스크린세이버 레이어 초기화
 */
void init_screensaver_clock(lv_style_t *style_time, lv_style_t *style_date) {
    last_user_interaction = millis();
    
    screensaver_layer = lv_obj_create(lv_layer_top());
    lv_obj_set_size(screensaver_layer, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_bg_color(screensaver_layer, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(screensaver_layer, LV_OPA_100, 0);
    lv_obj_set_style_border_width(screensaver_layer, 0, 0);
    
    lv_obj_add_flag(screensaver_layer, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(screensaver_layer, screensaver_click_cb, LV_EVENT_CLICKED, NULL);

    // 1. 시계 라벨 (중앙)
    screensaver_clock_label = lv_label_create(screensaver_layer);
    if (style_time) lv_obj_add_style(screensaver_clock_label, style_time, 0);
    
    lv_obj_set_style_text_color(screensaver_clock_label, lv_color_hex(0xFFFFFF), 0);
    lv_label_set_text(screensaver_clock_label, "00:00");
    lv_obj_center(screensaver_clock_label);

    // 2. 날짜 라벨 (시계 위쪽)
    screensaver_date_label = lv_label_create(screensaver_layer);
    if (style_date) lv_obj_add_style(screensaver_date_label, style_date, 0);
    
    lv_obj_set_style_text_color(screensaver_date_label, lv_color_hex(0xAAAAAA), 0); // 약간 회색
    lv_label_set_text(screensaver_date_label, "YY. MM. DD. (DAY)");
    
    // 위치 조정: 시계 위로 60픽셀 이동
    lv_obj_align_to(screensaver_date_label, screensaver_clock_label, LV_ALIGN_OUT_TOP_MID, 0, -20);

    lv_obj_add_flag(screensaver_layer, LV_OBJ_FLAG_HIDDEN);
    is_screensaver_active = false;
}

/**
 * 시계 및 날짜 업데이트 (데이터 수신 시 호출)
 */
void update_screensaver_time(int year, int month, int day, int weekday, int hour, int minute) {
    if (screensaver_clock_label == nullptr || screensaver_date_label == nullptr) return;
    
    static int last_m = -1;
    static int last_d = -1;

    // 분이 바뀌었을 때만 갱신 (효율성)
    if (last_m != minute || last_d != day) { 
        last_m = minute;
        last_d = day;

        // 시간 업데이트
        lv_label_set_text_fmt(screensaver_clock_label, "%02d:%02d", hour, minute);

        // [수정] 날짜 업데이트 (YY. MM. DD. (DAY))
        const char* day_str = (weekday >= 0 && weekday <= 6) ? WEEKDAY_STR[weekday] : "---";
        lv_label_set_text_fmt(screensaver_date_label, "%02d. %02d. %02d. (%s)", year, month, day, day_str);
        
        // 날짜가 변경되면 위치 재정렬 (글자 길이에 따라 중앙 정렬 유지)
        lv_obj_align_to(screensaver_date_label, screensaver_clock_label, LV_ALIGN_OUT_TOP_MID, 0, -20);
    }
}

void process_screensaver_timer() {
    // [수정] 스크린세이버 대기 시간 단축: 30초 -> 15초
    const uint32_t SCREENSAVER_TIMEOUT = 15000; 
    
    if (!is_screensaver_active && (millis() - last_user_interaction > SCREENSAVER_TIMEOUT)) {
        if (screensaver_layer != nullptr) {
            lv_obj_clear_flag(screensaver_layer, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(screensaver_layer);
            is_screensaver_active = true;
        }
    }
}