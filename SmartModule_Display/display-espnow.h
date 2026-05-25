#pragma once

#include "esphome.h"
#include "esphome/core/application.h"
#include "esphome/components/light/light_state.h"
#include "esphome/components/number/number.h"
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <vector>
#include <cstring>
#include <string>
#include "display-ui-helpers.h"

// ESP-IDF 버전 확인용
#if defined(ESP_PLATFORM)
  #include <esp_idf_version.h>
#endif

class DisplayESPNow;
extern DisplayESPNow *global_espnow;

// 브로드캐스트 주소
static uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

class DisplayESPNow : public esphome::Component {
 private:
  lv_obj_t *log_label = nullptr;
  
  bool initialized = false;
  uint32_t last_init_attempt = 0;
  
  // 페어링 상태
  bool pairing_mode = false;
  uint32_t last_pairing_blink_time = 0;
  uint32_t last_pairing_packet_time = 0;
  bool pairing_led_state = false;

  // 하트비트 및 연결 상태
  uint32_t last_heartbeat_time = 0;
  uint32_t last_valid_packet_time = 0; 

  // 피어 정보
  uint8_t current_peer_mac[6] = {0, 0, 0, 0, 0, 0};
  bool has_peer = false;

  // 송/수신 상태
  volatile esp_now_send_status_t last_send_status = ESP_NOW_SEND_FAIL;
  volatile bool isr_send_flag_fired = false; 
  volatile bool isr_recv_flag_fired = false;
  volatile int last_recv_len = 0; 
  FeedbackData recv_buffer;
  uint8_t last_sender_mac[6]; 

  char my_mac_str[20] = "--:--:--:--:--:--";
  char peer_mac_str[20] = "--:--:--:--:--:--"; 

  // 오토라이트 상태 추적용
  bool current_auto_light_state = false;

 public:
  void set_log_label(lv_obj_t *label) {
    this->log_label = label;
    if (label) {
        lv_label_set_text(label, "Loading...");
        lv_obj_set_style_text_color(label, lv_color_hex(0x888888), 0);
    }
  }

  void setup() override {
    ESP_LOGE("custom_espnow", ">>> Component setup STARTED <<<");
    last_init_attempt = 0; 
    
    // 내 MAC 로드
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(my_mac_str, sizeof(my_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
    // 채널 로드
    int ch = id(wifi_channel_storage);
    if (ch < 1 || ch > 13) ch = 1;

    // 피어 MAC 로드
    std::string saved_mac = id(peer_mac_storage);
    if (saved_mac.length() >= 17) { 
        int values[6];
        if (sscanf(saved_mac.c_str(), "%x:%x:%x:%x:%x:%x", 
                   &values[0], &values[1], &values[2], 
                   &values[3], &values[4], &values[5]) == 6) {
            bool is_zero = true;
            for(int i=0; i<6; i++) if(values[i] != 0) is_zero = false;
            if (!is_zero) {
                for(int i=0; i<6; i++) current_peer_mac[i] = (uint8_t)values[i];
                has_peer = true;
                snprintf(peer_mac_str, sizeof(peer_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
                         current_peer_mac[0], current_peer_mac[1], current_peer_mac[2], 
                         current_peer_mac[3], current_peer_mac[4], current_peer_mac[5]);
            }
        }
    }

    App.scheduler.set_interval(this, "espnow_manual_loop", 20, [this]() {
        this->loop();
    });
  }

  void loop() override {
    uint32_t now = millis();

    // 1. 초기화
    if (!initialized) {
        if (now - last_init_attempt > 5000) {
            last_init_attempt = now;
            if (try_init_espnow()) {
                initialized = true;
                if (has_peer) {
                    add_peer_if_not_exists(current_peer_mac);
                    update_peer_ui(true);
                } else {
                    update_peer_ui(false);
                }
            }
        }
        return; 
    }

    // 2. 송신 완료 플래그
    if (isr_send_flag_fired) {
        isr_send_flag_fired = false; 
        if (last_send_status == ESP_NOW_SEND_SUCCESS) {
            last_valid_packet_time = millis();
        }
        update_link_status_ui(); 
    }

    // 3. 수신 처리
    if (isr_recv_flag_fired) {
        isr_recv_flag_fired = false;
        last_valid_packet_time = millis();
        update_link_status_ui(); 

        if (last_recv_len == sizeof(FeedbackData)) {
            if (recv_buffer.packet_type == 0xA2) {
                 process_feedback_data(&recv_buffer);

                 // 페어링 로직
                 if (pairing_mode || !has_peer) {
                     finish_pairing_success(last_sender_mac);
                 } else {
                     if (memcmp(current_peer_mac, last_sender_mac, 6) != 0) {
                         finish_pairing_success(last_sender_mac); 
                     }
                 }
            } 
        }
    }

    // 4. Heartbeat
    if (initialized && has_peer && !pairing_mode) {
        if (now - last_heartbeat_time > 3000) {
            last_heartbeat_time = now;
            send_heartbeat_unicast();
        }
    }

    // 5. 연결 상태 LED UI 갱신 (1초)
    static uint32_t last_check = 0;
    if (now - last_check > 1000) {
        last_check = now;
        update_link_status_ui();
    }

    // 6. 페어링 LED 처리
    if (pairing_mode) {
        if (now - last_pairing_blink_time > 1000) {
            last_pairing_blink_time = now;
            pairing_led_state = !pairing_led_state;
            if (log_label) {
                if (pairing_led_state) {
                     lv_obj_set_style_text_color(log_label, lv_color_hex(0x0095FF), 0);
                     lv_label_set_text(log_label, "PAIRING...");
                } else {
                     lv_obj_set_style_text_color(log_label, lv_color_hex(0x444444), 0);
                }
            }
        }
        if (now - last_pairing_packet_time > 600) {
            last_pairing_packet_time = now;
            send_pairing_signal_broadcast();
        }
    }
  }

  void process_feedback_data(FeedbackData *fData) {
      // 1. 인디케이터 UI 업데이트
      id(fl_heat_level) = fData->fl_heat;
      id(fl_vent_level) = fData->fl_vent;
      id(fr_heat_level) = fData->fr_heat;
      id(fr_vent_level) = fData->fr_vent;
      id(ll_heat_level) = fData->rl_heat;
      id(lr_heat_level) = fData->rr_heat;
      
      lv_color_t heat_color = lv_color_hex(0xFF8000); 
      lv_color_t vent_color = lv_color_hex(0x0095FF); 
      lv_color_t steer_color = lv_color_hex(0xFF8000);

      update_indicator(&id(fl_ind_1), id(fl_heat_level) >= 1, heat_color);
      update_indicator(&id(fl_ind_2), id(fl_heat_level) >= 2, heat_color);
      update_indicator(&id(fl_ind_3), id(fl_heat_level) >= 3, heat_color);
      update_indicator(&id(fl_vent_ind_1), id(fl_vent_level) >= 1, vent_color);
      update_indicator(&id(fl_vent_ind_2), id(fl_vent_level) >= 2, vent_color);
      update_indicator(&id(fl_vent_ind_3), id(fl_vent_level) >= 3, vent_color);

      update_indicator(&id(fr_ind_1), id(fr_heat_level) >= 1, heat_color);
      update_indicator(&id(fr_ind_2), id(fr_heat_level) >= 2, heat_color);
      update_indicator(&id(fr_ind_3), id(fr_heat_level) >= 3, heat_color);
      update_indicator(&id(fr_vent_ind_1), id(fr_vent_level) >= 1, vent_color);
      update_indicator(&id(fr_vent_ind_2), id(fr_vent_level) >= 2, vent_color);
      update_indicator(&id(fr_vent_ind_3), id(fr_vent_level) >= 3, vent_color);
      
      id(steer_heat_state) = (fData->steer_heat == 1); 
      update_indicator(&id(ind_steer), id(steer_heat_state), steer_color);

      update_indicator(&id(ll_ind_1), id(ll_heat_level) >= 1, heat_color);
      update_indicator(&id(ll_ind_2), id(ll_heat_level) >= 2, heat_color);
      update_indicator(&id(lr_ind_1), id(lr_heat_level) >= 1, heat_color);
      update_indicator(&id(lr_ind_2), id(lr_heat_level) >= 2, heat_color);
      
      id(driver_state) = (fData->driver_only == 1);
      id(ac_state) = (fData->ac == 1);
      id(heat_state) = (fData->heat == 1);
      id(sync_state) = (fData->sync == 1);
      
      update_text_glow(&id(lbl_driver), id(driver_state), vent_color);
      update_text_glow(&id(lbl_ac), id(ac_state), vent_color);
      update_text_glow(&id(lbl_heat), id(heat_state), heat_color);
      update_text_glow(&id(lbl_sync), id(sync_state), lv_color_hex(0x2DA041));

      // 2. 날짜 및 시계 업데이트
      update_screensaver_time(fData->year, fData->month, fData->day, fData->weekday, fData->hour, fData->minute);

      // 3. 오토라이트 밝기 제어
      bool new_auto_state = (fData->auto_light == 1);
      
      if (current_auto_light_state != new_auto_state) {
          current_auto_light_state = new_auto_state;
          
          float target_bright = 1.0f;
          if (current_auto_light_state) {
              // 0~100 값을 0.0~1.0으로 변환
              target_bright = id(dim_brightness_level).state / 100.0f; 
          } else {
              target_bright = 1.0f; 
          }
          
          auto call = id(display_backlight).make_call();
          call.set_brightness(target_bright);
          call.set_transition_length(1000); 
          call.perform();
      }
  }

  // --- 기존 ESP-NOW 연결/페어링 관련 함수들은 유지 ---
  void start_pairing_mode() {
      if (pairing_mode) return; 
      pairing_mode = true; last_pairing_blink_time = 0; last_pairing_packet_time = 0;
      force_channel_safe();
      if (has_peer) {
          if (esp_now_is_peer_exist(current_peer_mac)) esp_now_del_peer(current_peer_mac);
          has_peer = false; memset(current_peer_mac, 0, 6);
      }
  }
  void stop_pairing_mode() { pairing_mode = false; update_peer_ui(true); }
  
  void finish_pairing_success(const uint8_t *new_mac) {
      memcpy(current_peer_mac, new_mac, 6); has_peer = true;
      add_peer_if_not_exists(current_peer_mac);
      snprintf(peer_mac_str, sizeof(peer_mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
               new_mac[0], new_mac[1], new_mac[2], new_mac[3], new_mac[4], new_mac[5]);
      id(peer_mac_storage) = std::string(peer_mac_str); 
      if (pairing_mode) stop_pairing_mode(); else update_peer_ui(true);
  }

  void update_peer_ui(bool is_connected) {
      if (log_label == nullptr) return;
      char buf[32];
      if (has_peer) snprintf(buf, sizeof(buf), "Peer: %s", peer_mac_str);
      else snprintf(buf, sizeof(buf), "Me: %s", my_mac_str);
      lv_label_set_text(log_label, buf);
      update_link_status_ui();
  }

  void update_link_status_ui() {
    if (pairing_mode || log_label == nullptr) return;
    bool is_alive = (millis() - last_valid_packet_time < 5000);
    lv_obj_set_style_text_color(log_label, is_alive ? lv_color_hex(0x2DA041) : lv_color_hex(0x888888), 0);
  }

  void force_channel_safe() {
    int target_ch = id(wifi_channel_storage);
    if (target_ch < 1 || target_ch > 13) target_ch = 1;
    uint8_t p; wifi_second_chan_t s;
    esp_wifi_get_channel(&p, &s);
    if (p != target_ch) {
       esp_wifi_set_promiscuous(true);
       esp_wifi_set_channel(target_ch, WIFI_SECOND_CHAN_NONE);
       esp_wifi_set_promiscuous(false);
    }
  }

  bool try_init_espnow() {
    wifi_mode_t mode;
    if (esp_wifi_get_mode(&mode) != ESP_OK || mode == WIFI_MODE_NULL) return false;
    if (esp_now_init() != ESP_OK) return false;
    esp_now_unregister_recv_cb(); esp_now_unregister_send_cb();
    #if (defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3) || \
        (defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0))
      esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
    #else
      esp_now_register_recv_cb(OnDataRecv);
    #endif
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
    force_channel_safe(); add_peer_if_not_exists(BROADCAST_MAC);
    return true;
  }

  void add_peer_if_not_exists(const uint8_t *mac) {
    int target_ch = id(wifi_channel_storage); if (target_ch < 1) target_ch = 1;
    uint8_t current_ch; wifi_second_chan_t second; esp_wifi_get_channel(&current_ch, &second);
    uint8_t final_ch = (current_ch > 0) ? current_ch : target_ch;
    if (esp_now_is_peer_exist(mac)) {
        bool is_bc = true; for(int i=0;i<6;i++) if(mac[i]!=0xFF) is_bc=false;
        if (!is_bc) {
            esp_now_peer_info_t pi;
            if (esp_now_get_peer(mac, &pi) == ESP_OK) {
                if (pi.channel != final_ch) esp_now_del_peer(mac); else return;
            }
        } else return;
    }
    esp_now_peer_info_t peerInfo = {}; memcpy(peerInfo.peer_addr, mac, 6);
    peerInfo.channel = final_ch; peerInfo.encrypt = false; peerInfo.ifidx = WIFI_IF_AP; 
    esp_now_add_peer(&peerInfo);
  }

  void send_pairing_signal_broadcast() {
    PairingData p_data; p_data.packet_type = 0xB1;
    esp_read_mac(p_data.mac_addr, ESP_MAC_WIFI_SOFTAP);
    force_channel_safe(); add_peer_if_not_exists(BROADCAST_MAC);
    esp_now_send(BROADCAST_MAC, (uint8_t *) &p_data, sizeof(PairingData));
  }

  void send_heartbeat_unicast() {
    if (!initialized || !has_peer) return; 
    PairingData p_data; p_data.packet_type = 0xB1;
    esp_read_mac(p_data.mac_addr, ESP_MAC_WIFI_SOFTAP);
    force_channel_safe(); add_peer_if_not_exists(current_peer_mac);
    esp_now_send(current_peer_mac, (uint8_t *) &p_data, sizeof(PairingData));
  }

  void send_hvac_data(HvacData &data) {
    if (!initialized || !has_peer) return; 
    force_channel_safe(); add_peer_if_not_exists(current_peer_mac);
    esp_now_send(current_peer_mac, (uint8_t *) &data, sizeof(HvacData));
  }

  #if (defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3) || \
      (defined(ESP_IDF_VERSION) && ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0))
  static void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingData, int len) {
      const uint8_t *mac = info->src_addr;
  #else
  static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  #endif
      if (global_espnow == nullptr || incomingData == nullptr || len <= 0) return;
      int copy_len = (len > sizeof(FeedbackData)) ? sizeof(FeedbackData) : len;
      memcpy((void*)&global_espnow->recv_buffer, incomingData, copy_len);
      if (mac != nullptr) memcpy(global_espnow->last_sender_mac, mac, 6);
      global_espnow->last_recv_len = len; 
      global_espnow->isr_recv_flag_fired = true; 
  }

  static void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (global_espnow) {
        global_espnow->last_send_status = status; 
        global_espnow->isr_send_flag_fired = true;     
    }
  }
};

DisplayESPNow *global_espnow = nullptr;