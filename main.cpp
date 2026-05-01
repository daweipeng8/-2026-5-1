#include <Arduino.h>
#include "usb/usb_host.h"

// ========== 引脚定义 ==========
#define HALL1_PIN      2    // 霍尔1，低电平触发
#define HALL2_PIN      42   // 霍尔2，低电平触发
#define PRESSURE_PIN   1    // 压敏电阻，数字量输入
#define SWITCH_PIN     4    // 开关，切换高低电平
#define BUTTON_PIN     21   // 按键

// ========== 去抖时间(ms) ==========
#define DEBOUNCE_MS    50

// ========== 各输入状态 ==========
static bool hall1_last = HIGH;
static bool hall2_last = HIGH;
static bool pressure_last = LOW;
static bool switch_last = HIGH;
static bool button_last = HIGH;

static unsigned long hall1_time = 0;
static unsigned long hall2_time = 0;
static unsigned long pressure_time = 0;
static unsigned long switch_time = 0;
static unsigned long button_time = 0;

// ========== USB 键盘相关 ==========
static usb_host_client_handle_t client_hdl = NULL;
static usb_device_handle_t dev_hdl = NULL;
static usb_transfer_t *xfer_in = NULL;
static uint8_t kbd_intf = 0;
static volatile bool kbd_ready = false;
static uint8_t last_keys[6] = {0};

// ========== 去抖读取函数 ==========
bool debounce_read(int pin, bool &last_state, unsigned long &last_time) {
  bool raw = digitalRead(pin);
  if (raw != last_state) {
    if (millis() - last_time > DEBOUNCE_MS) {
      last_time = millis();
      last_state = raw;
      return true;  // 状态变化了
    }
  }
  return false;
}

// ========== USB Host 后台任务 ==========
static void host_task(void *arg) {
  while (1) {
    uint32_t flags;
    usb_host_lib_handle_events(portMAX_DELAY, &flags);
  }
}

// ========== USB 传输回调 ==========
static void xfer_cb(usb_transfer_t *t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes >= 3) {
    uint8_t *d = t->data_buffer;
    // 检查是否有新按键按下（对比上次）
    for (int i = 2; i < 8 && i < t->actual_num_bytes; i++) {
      if (d[i] == 0) continue;
      // 看这个键在上次报告里有没有
      bool is_new = true;
      for (int j = 0; j < 6; j++) {
        if (last_keys[j] == d[i]) { is_new = false; break; }
      }
      if (is_new) {
        Serial.println("KB_PRESS");
      }
    }
    // 保存本次按键状态
    for (int i = 0; i < 6; i++) {
      last_keys[i] = (i + 2 < t->actual_num_bytes) ? d[i + 2] : 0;
    }
  }
  if (kbd_ready) usb_host_transfer_submit(t);
}

// ========== USB 客户端回调 ==========
static void client_cb(const usb_host_client_event_msg_t *msg, void *arg) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
    Serial.printf("[USB] 设备连接 addr=%d\n", msg->new_dev.address);
    if (usb_host_device_open(client_hdl, msg->new_dev.address, &dev_hdl) != ESP_OK) {
      Serial.println("[USB] 打开设备失败");
      return;
    }

    const usb_config_desc_t *cfg;
    usb_host_get_active_config_descriptor(dev_hdl, &cfg);

    const uint8_t *p = (const uint8_t *)cfg;
    int off = 0, total = cfg->wTotalLength;
    bool is_kbd = false;
    uint8_t ep_addr = 0;
    int ep_mps = 0;

    while (off < total) {
      if (p[off] == 0) break;
      if (p[off + 1] == 0x04) {
        is_kbd = (p[off + 5] == 3 && p[off + 7] == 1);
        if (is_kbd) kbd_intf = p[off + 2];
      }
      if (p[off + 1] == 0x05 && is_kbd && (p[off + 2] & 0x80)) {
        ep_addr = p[off + 2];
        ep_mps = p[off + 4] | (p[off + 5] << 8);
        break;
      }
      off += p[off];
    }

    if (!ep_addr) {
      Serial.println("[USB] 未找到键盘端点");
      usb_host_device_close(client_hdl, dev_hdl);
      dev_hdl = NULL;
      return;
    }

    Serial.printf("[USB] 键盘就绪 intf=%d ep=0x%02X mps=%d\n", kbd_intf, ep_addr, ep_mps);
    usb_host_interface_claim(client_hdl, dev_hdl, kbd_intf, 0);

    usb_host_transfer_alloc(ep_mps, 0, &xfer_in);
    xfer_in->device_handle = dev_hdl;
    xfer_in->bEndpointAddress = ep_addr;
    xfer_in->callback = xfer_cb;
    xfer_in->num_bytes = ep_mps;

    memset(last_keys, 0, sizeof(last_keys));
    kbd_ready = true;
    usb_host_transfer_submit(xfer_in);

  } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
    Serial.println("[USB] 设备断开");
    kbd_ready = false;
    if (dev_hdl) {
      usb_host_interface_release(client_hdl, dev_hdl, kbd_intf);
      usb_host_device_close(client_hdl, dev_hdl);
      dev_hdl = NULL;
    }
    if (xfer_in) {
      usb_host_transfer_free(xfer_in);
      xfer_in = NULL;
    }
  }
}

// ========== setup ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("======= 系统启动 =======");

  // GPIO 初始化
  pinMode(HALL1_PIN, INPUT_PULLUP);
  pinMode(HALL2_PIN, INPUT_PULLUP);
  pinMode(PRESSURE_PIN, INPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // 读取初始状态
  hall1_last = digitalRead(HALL1_PIN);
  hall2_last = digitalRead(HALL2_PIN);
  pressure_last = digitalRead(PRESSURE_PIN);
  switch_last = digitalRead(SWITCH_PIN);
  button_last = digitalRead(BUTTON_PIN);

  Serial.printf("[初始状态] 霍尔1=%s 霍尔2=%s 压敏=%d 开关=%s 按键=%s\n",
    hall1_last == LOW ? "触发" : "未触发",
    hall2_last == LOW ? "触发" : "未触发",
    pressure_last,
    switch_last == LOW ? "ON" : "OFF",
    button_last == LOW ? "按下" : "松开");

  // USB Host 初始化
  usb_host_config_t hcfg = {};
  hcfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
  esp_err_t err = usb_host_install(&hcfg);
  if (err != ESP_OK) {
    Serial.printf("[USB] 安装失败: 0x%X\n", err);
  } else {
    xTaskCreatePinnedToCore(host_task, "usb_host", 4096, NULL, 2, NULL, 0);

    usb_host_client_config_t ccfg = {};
    ccfg.max_num_event_msg = 5;
    ccfg.async.client_event_callback = client_cb;
    usb_host_client_register(&ccfg, &client_hdl);
    Serial.println("[USB] 等待键盘连接...");
  }

  Serial.println("======= 就绪 =======");
}

// ========== loop ==========
void loop() {
  // --- USB 事件处理 ---
  if (client_hdl) {
    usb_host_client_handle_events(client_hdl, 1);
  }

  // --- 霍尔1 (IO2, 低电平触发) ---
  if (debounce_read(HALL1_PIN, hall1_last, hall1_time)) {
    if (hall1_last == LOW) {
      Serial.println("HALL1_ON");
    } else {
      Serial.println("HALL1_OFF");
    }
  }

  // --- 霍尔2 (IO42, 低电平触发) ---
  if (debounce_read(HALL2_PIN, hall2_last, hall2_time)) {
    if (hall2_last == LOW) {
      Serial.println("HALL2_ON");
    } else {
      Serial.println("HALL2_OFF");
    }
  }

  // --- 压敏电阻 (IO1, 电平触发) ---
  if (debounce_read(PRESSURE_PIN, pressure_last, pressure_time)) {
    if (pressure_last == HIGH) {
      Serial.println("PRESSURE_ON");
    } else {
      Serial.println("PRESSURE_OFF");
    }
  }

  // --- 开关 (IO4, 切换高低电平) ---
  if (debounce_read(SWITCH_PIN, switch_last, switch_time)) {
    if (switch_last == LOW) {
      Serial.println("SWITCH_ON");
    } else {
      Serial.println("SWITCH_OFF");
    }
  }

  // --- 按键 (IO21, 按下松开) ---
  if (debounce_read(BUTTON_PIN, button_last, button_time)) {
    if (button_last == LOW) {
      Serial.println("BTN_PRESS");
    } else {
      Serial.println("BTN_RELEASE");
    }
  }
}