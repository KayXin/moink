/*
 * 墨印 · MoInk — 固件主装配（R 系列，重构版）
 *
 * 职责一句话：固件只做「热点 / 接收 / 刷图 / 休眠 / 唤醒 / OTA」，画面全部由页面
 * 在浏览器里合成。没有任何 STA 配网、没有拉图、没有固件端图像处理。
 *
 * 模块分工：
 *   epd_drv   屏驱动（A0/A1；引脚 R1.2.2 起运行时可配）
 *   settings  持久化：面板 / 翻转 / GPIO 引脚 / 休眠时长 / 自动唤醒 / 热点凭据
 *   netif_ap  纯 SoftAP（默认开放，SSID 默认 MoInk-XXXX）
 *   captive   DNS 劫持 + OS 探测劫持（连上即弹控制页）
 *   frame     帧接收（16B 头 + CRC16 校验）+ 显示触发
 *   power     电池 ADC / 深睡 / 唤醒源 / 空闲休眠
 *   ota_web   OTA 双槽 + 控制页热更（web 分区）+ 统一升级入口
 *
 * 路由：
 *   GET  /                控制页（web 分区优先，否则内嵌页）
 *   GET  /api/info        状态（版本 / 接口 / 面板 / 堆 / 电量 / 客户端数）
 *   GET/POST /api/settings 读写设置
 *   POST /api/frame       传图
 *   POST /api/upload      ★统一升级入口（自动识别 .bin 固件 / .html 控制页）
 *   POST /api/web/clear   清除已上传控制页，回退内嵌页
 *   POST /api/clear       残影清理
 *   POST /api/reboot      重启设备（GPIO 设置改后由页面调用）
 *   POST /api/factory     恢复出厂
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "version.h"
#include "epd_drv.h"
#include "settings.h"
#include "netif_ap.h"
#include "captive.h"
#include "frame.h"
#include "power.h"
#include "ota_web.h"
#include "store.h"

#include "index_html.h"     /* 由 tools/make_index_header.py 固化，改页必重跑 */

static const char *TAG = "main";

#define HOLD_RESET_S    5          /* 长按恢复出厂 */
#define HTTPD_STACK     8192       /* R1.3.1：轮播 list 组装 base64 需要更大栈 */
#define HTTPD_MAX_URI   32         /* R1.3.1：16 条业务路由 + OTA + captive 兜底约 19 个，
                                       默认 8 会满（ESP_ERR_HTTPD_HANDLERS_FULL → abort 循环） */

/* R1.3.0：GPIO 唤醒 = 用户按过键。唤醒的这次按下也算一次短按（切图），
   由 button_task 消费（见其注释）。 */
static bool s_woken_by_btn = false;

/* ---------------- 请求小工具 ---------------- */

static void url_decode(char *s)
{
    char *r = s, *w = s;
    while (*r) {
        if (*r == '+') { *w++ = ' '; r++; }
        else if (*r == '%' && r[1] && r[2]) {
            int hi = 0, lo = 0;
            char h = r[1], l = r[2];
            hi = (h <= '9') ? h - '0' : ((h | 0x20) - 'a' + 10);
            lo = (l <= '9') ? l - '0' : ((l | 0x20) - 'a' + 10);
            *w++ = (char)((hi << 4) | lo);
            r += 3;
        } else { *w++ = *r++; }
    }
    *w = '\0';
}

/* 从 key=value&... 串里取值，命中返回 1，值经 URL 解码写入 out。 */
static int kv_get(const char *qs, const char *key, char *out, size_t outlen)
{
    size_t klen = strlen(key);
    const char *p = qs;
    while (p && *p) {
        const char *amp = strchr(p, '&');
        size_t seg = amp ? (size_t)(amp - p) : strlen(p);
        if (seg > klen && strncmp(p, key, klen) == 0 && p[klen] == '=') {
            size_t vlen = seg - klen - 1;
            if (vlen >= outlen) vlen = outlen - 1;
            memcpy(out, p + klen + 1, vlen);
            out[vlen] = '\0';
            url_decode(out);
            return 1;
        }
        p = amp ? amp + 1 : NULL;
    }
    return 0;
}

/* 读请求体到栈缓冲（限长），成功返回长度。
   R1.0.8：httpd_req_recv 单次可能短读（TCP 分段），必须循环收满；
   超出缓冲的部分读走丢弃，避免残体污染 keep-alive 的下一个请求。 */
static int read_body(httpd_req_t *req, char *buf, size_t n)
{
    size_t len = req->content_len;
    if (len == 0) return 0;
    size_t cap = (len > n - 1) ? n - 1 : len;
    size_t got = 0;
    while (got < len) {
        if (got < cap) {
            int k = httpd_req_recv(req, buf + got, cap - got);
            if (k <= 0) break;
            got += (size_t)k;
        } else {
            char sink[64];                    /* 溢出部分丢弃 */
            int k = httpd_req_recv(req, sink, sizeof(sink));
            if (k <= 0) break;
            got += (size_t)k;
        }
    }
    if (got == 0) return 0;
    size_t kept = (got < cap) ? got : cap;
    buf[kept] = '\0';
    return (int)kept;
}

/* ---------------- 控制页 ---------------- */

static esp_err_t page_handler(httpd_req_t *req)
{
    power_activity();

    if (ota_web_has_page()) return ota_web_stream_page(req);

    /* 页面随固件一起更新，浏览器缓存会让人误判「没生效」——必须禁缓存。
       热更页那条路径在 ota_web_stream_page() 里同样设了 no-store。 */
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_send(req, (const char *)index_html, index_html_len);
}

/* ---------------- /api/info ---------------- */

static esp_err_t info_handler(httpd_req_t *req)
{
    power_activity();
    const moink_settings_t *s = settings_get();
    char buf[512];

    /* ver = 统一版本号 = 当前生效页面版本（热更页优先，否则等于固件编译号）。
       fw  = 固件编译进去的号：仅供排障，以及判断「热更控制页是否落后于固件」
             （ver < fw 时页面给黄色警示），界面只显示 ver。
       api 仍是帧格式契约号，与发布版本号解耦，页面状态栏另起一行显示。
       store_slots / store_count / store_cur：轮播容量与当前状态（R1.3.0）。 */
    int caro_count = 0, caro_cur = 0;
    store_meta_get(&caro_count, &caro_cur);
    int n = snprintf(buf, sizeof(buf),
        "{\"ver\":\"%s\",\"fw\":\"%s\",\"api\":%d,"
        "\"panel\":\"%s\",\"gpio_preset\":%u,\"heap\":%lu,\"bat_mv\":%d,"
        "\"sleep_s\":%lu,\"wake_s\":%lu,\"clients\":%d,"
        "\"ssid\":\"%s\",\"uptime\":%lld,\"store_slots\":%d,"
        "\"store_count\":%d,\"store_cur\":%d}",
        ota_web_page_version(), MOINK_VERSION, MOINK_API_VERSION,
        epd_panel_name((epd_panel_t)s->panel), (unsigned)s->gpio_preset,
        (unsigned long)esp_get_free_heap_size(), power_battery_mv(),
        (unsigned long)s->sleep_s, (unsigned long)s->wake_s,
        netif_ap_client_count(), netif_ap_ssid(),
        (long long)(esp_timer_get_time() / 1000000), store_slots(),
        caro_count, caro_cur);
    if (n < 0 || n >= (int)sizeof(buf)) buf[sizeof(buf) - 1] = '\0';

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* ---------------- /api/settings ---------------- */

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    power_activity();
    const moink_settings_t *s = settings_get();
    char buf[512];

    snprintf(buf, sizeof(buf),
        "{\"panel\":%u,\"hflip\":%u,\"wifi_pwr\":%u,\"gpio_preset\":%u,"
        "\"pins\":[%u,%u,%u,%u,%u,%u,%u,%u],"
        "\"sleep_s\":%lu,\"wake_s\":%lu,\"ssid\":\"%s\",\"pass_set\":%s}",
        s->panel, s->hflip, s->wifi_pwr, s->gpio_preset,
        s->pins.v[0], s->pins.v[1], s->pins.v[2], s->pins.v[3],
        s->pins.v[4], s->pins.v[5], s->pins.v[6], s->pins.v[7],
        (unsigned long)s->sleep_s, (unsigned long)s->wake_s,
        s->ap_ssid, s->ap_pass[0] ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, strlen(buf));
}

/* FB-014②：AP 配置延迟应用 —— 先把 200 应答发回给页面，300ms 后再把新配置
   应用到热点。改名 / 改密码会立刻踢掉手机连接，若「先应用后应答」，页面大概率
   收不到 200，成败只能靠回读猜；应答先行后，保存判定走正常成功路径。
   esp_timer 一次性回调，绝不在 httpd 任务栈里等。 */
static esp_timer_handle_t s_ap_timer;

static void ap_apply_timer_cb(void *arg)
{
    (void)arg;
    netif_ap_apply();
}

static esp_err_t settings_post_handler(httpd_req_t *req)
{
    power_activity();
    char body[512];
    read_body(req, body, sizeof(body));

    char v[SETT_PASS_MAX];
    bool ap_changed = false;
    bool panel_changed = false, hflip_changed = false;

    if (kv_get(body, "panel", v, sizeof(v))) {
        settings_set_panel((uint8_t)atoi(v));
        panel_changed = true;
    }
    if (kv_get(body, "hflip", v, sizeof(v))) {
        settings_set_hflip((uint8_t)atoi(v));
        hflip_changed = true;
    }
    /* GPIO 设置（R1.2.2）：preset 0/1/2 用固定预设，3 = 自定义（须带 8 个引脚）。
       引脚变更只落盘、不改运行时——SPI 总线与 ADC 通道在启动时按引脚初始化，
       运行中重映射会把屏驱动搞死；由页面引导用户重启生效。 */
    bool gpio_changed = false;
    if (kv_get(body, "gpio_preset", v, sizeof(v))) {
        uint8_t preset = (uint8_t)atoi(v);
        if (preset == GPIO_PRESET_CUSTOM) {
            static const char *PK[PIN_IDX_COUNT] = {
                "p_adc", "p_wake", "p_sck", "p_mosi", "p_cs", "p_dc", "p_rst", "p_busy"
            };
            pin_cfg_t c;
            bool have = true;
            for (int i = 0; i < PIN_IDX_COUNT; i++) {
                char pv[8];
                if (!kv_get(body, PK[i], pv, sizeof(pv))) { have = false; break; }
                c.v[i] = (uint8_t)atoi(pv);
            }
            if (!have)
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "custom pins incomplete");
            if (settings_set_gpio(GPIO_PRESET_CUSTOM, &c) != ESP_OK)
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "invalid pin map");
        } else if (settings_set_gpio(preset, NULL) != ESP_OK) {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad gpio preset");
        }
        gpio_changed = true;
    }
    if (kv_get(body, "wifi_pwr", v, sizeof(v))) {
        settings_set_wifi_pwr((uint8_t)atoi(v));
        netif_ap_apply_tx_power();
    }
    if (kv_get(body, "sleep_s", v, sizeof(v))) {
        settings_set_sleep((uint32_t)atoi(v));
    }
    if (kv_get(body, "wake_s", v, sizeof(v))) {
        settings_set_wake((uint32_t)atoi(v));
    }

    /* 热点凭据（FB-014①）：单字段可改 —— pass 键缺省 = 保持当前密码，
       pass 空串 = 清除密码（热点转开放）；ssid 键缺省 = 保持当前热点名。
       页面据此区分「留空不动」与「勾选清除」。 */
    char ssid[SETT_SSID_MAX], pass[SETT_PASS_MAX];
    bool have_ssid = kv_get(body, "ssid", ssid, sizeof(ssid));
    bool have_pass = kv_get(body, "pass", pass, sizeof(pass));
    if (have_ssid || have_pass) {
        settings_set_ap(have_ssid ? ssid : NULL, have_pass ? pass : NULL);
        ap_changed = true;
    }

    /* 应用到运行时。 */
    if (panel_changed) epd_set_panel((epd_panel_t)settings_get()->panel);
    if (hflip_changed) epd_set_hflip(settings_get()->hflip != 0);

    httpd_resp_set_type(req, "text/plain");
    esp_err_t err = httpd_resp_sendstr(req, gpio_changed ? "OK GPIO" : "OK");

    /* 应答已发出，再延迟应用 AP 配置（FB-014②）。 */
    if (ap_changed) {
        if (!s_ap_timer) {
            const esp_timer_create_args_t targs = {
                .callback = &ap_apply_timer_cb,
                .arg = NULL,
                .dispatch_method = ESP_TIMER_TASK,
                .name = "ap_apply",
            };
            esp_timer_create(&targs, &s_ap_timer);
        } else {
            esp_timer_stop(s_ap_timer);     /* 连续保存：重置 300ms 窗口 */
        }
        esp_timer_start_once(s_ap_timer, 300 * 1000);
    }
    return err;
}

/* ---------------- /api/clear / /api/factory ---------------- */

static esp_err_t clear_handler(httpd_req_t *req)
{
    power_activity();
    int cycles = 2;
    char v[16];
    if (kv_get(req->uri, "cycles", v, sizeof(v))) cycles = atoi(v);
    if (cycles < 1) cycles = 1;
    if (cycles > 4) cycles = 4;

    ESP_LOGI(TAG, "ghost clear x%d", cycles);
    int r = frame_clear_cycles(cycles);   /* R1.0.8：持帧锁，防与传图并发互踩 */
    if (r != 0) return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "clear failed");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t factory_handler(httpd_req_t *req)
{
    settings_factory_reset();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* R1.3.0：清空轮播（只清元数据，不擦 flash，槽位留待下次写入覆盖）。 */
static esp_err_t clear_carousel_handler(httpd_req_t *req)
{
    power_activity();
    store_clear();
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "OK");
}

/* ---------------- R1.3.1 轮播管理：list / delete / full ---------------- */

/* base64 分块编码并逐块发送：源数据按 3 字节/块编码为 4 字符，
   缓冲固定 ~5.5KB，避免为整图开 40KB 堆缓冲（C3 堆余量 8~26KB）。 */
static void b64_send_chunked(httpd_req_t *req, const uint8_t *in, size_t n)
{
    static const char T[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    char buf[4096 * 4 / 3 + 8];
    size_t off = 0;
    while (off < n) {
        size_t cnt = n - off;
        if (cnt > 4095) cnt = 4095;    /* 3 的倍数，避免跨块截断三元组 */
        size_t i = 0, o = 0;
        while (i + 3 <= cnt) {
            uint32_t v = ((uint32_t)in[off+i] << 16) | ((uint32_t)in[off+i+1] << 8) |
                         in[off+i+2];
            buf[o++] = T[(v >> 18) & 63]; buf[o++] = T[(v >> 12) & 63];
            buf[o++] = T[(v >> 6) & 63];  buf[o++] = T[v & 63];
            i += 3;
        }
        if (i + 1 == cnt) {
            uint32_t v = (uint32_t)in[off+i] << 16;
            buf[o++] = T[(v >> 18) & 63]; buf[o++] = T[(v >> 12) & 63];
            buf[o++] = '='; buf[o++] = '=';
        } else if (i + 2 == cnt) {
            uint32_t v = ((uint32_t)in[off+i] << 16) | ((uint32_t)in[off+i+1] << 8);
            buf[o++] = T[(v >> 18) & 63]; buf[o++] = T[(v >> 12) & 63];
            buf[o++] = T[(v >> 6) & 63];  buf[o++] = '=';
        }
        httpd_resp_send_chunk(req, buf, o);
        off += cnt;
    }
}

static esp_err_t carousel_list_handler(httpd_req_t *req)
{
    power_activity();
    int count = 0, cur = 0;
    store_meta_get(&count, &cur);

    char head[96];
    snprintf(head, sizeof(head), "{\"slots\":%d,\"count\":%d,\"cur\":%d,\"items\":[",
             store_slots(), count, cur);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send_chunk(req, head, strlen(head));

    char item[160];
    for (int i = 0; i < count; i++) {
        int fmt = 1, tw = 0, th = 0;
        uint32_t tlen = 0;
        const uint8_t *payload = NULL;
        uint8_t *dyn = NULL;

        /* 路径 A：预存缩略（R1.3.2/3）。直接读缩略头拿 fmt/tlen，
           按实际大小 malloc，避免按 20KB 上限一次性分配 */
        const esp_partition_t *sp = store_part();
        uint32_t off = (uint32_t)i * STORE_SLOT_SZ;
        uint8_t fhdr[16], thdr[16];
        int has_pre = 0;
        uint32_t flen = 0;
        if (sp &&
            esp_partition_read(sp, off, fhdr, sizeof(fhdr)) == ESP_OK &&
            fhdr[0] == STORE_MAGIC0 && fhdr[1] == STORE_MAGIC1) {
            flen = ((uint32_t)fhdr[8] << 24) | ((uint32_t)fhdr[9] << 16) |
                   ((uint32_t)fhdr[10] << 8) | fhdr[11];
            if (flen && flen <= STORE_FRAME_MAX &&
                esp_partition_read(sp, off + 16 + flen, thdr,
                                   sizeof(thdr)) == ESP_OK &&
                thdr[0] == STORE_MAGIC0 && thdr[1] == STORE_MAGIC1) {
                uint32_t tl = ((uint32_t)thdr[8] << 24) | ((uint32_t)thdr[9] << 16) |
                              ((uint32_t)thdr[10] << 8) | thdr[11];
                uint32_t lim = (thdr[2] == 2) ? STORE_THUMB_JPEG_MAX
                                               : STORE_THUMB_LEN;
                if (tl && tl <= lim) {
                    fmt = (thdr[2] == 2) ? 2 : 1;
                    tw = (int)((thdr[4] << 8) | thdr[5]);
                    th = (int)((thdr[6] << 8) | thdr[7]);
                    tlen = tl;
                    has_pre = 1;
                }
            }
        }
        if (!has_pre) {
            /* 路径 B：回退固件现算 2bit 缩略 */
            dyn = malloc(STORE_THUMB_ENTRY);
            if (!dyn) { fmt = 1; tw = 0; th = 0; tlen = 0; }
            else {
                int n = store_thumb_frame(i, dyn, STORE_THUMB_ENTRY);
                if (n <= 16) {
                    free(dyn); dyn = NULL;
                    fmt = 1; tw = 0; th = 0; tlen = 0;
                } else {
                    fmt = 1;
                    tw = (int)((dyn[4] << 8) | dyn[5]);
                    th = (int)((dyn[6] << 8) | dyn[7]);
                    tlen = (uint32_t)(n - 16);
                    payload = dyn + 16;
                }
            }
        } else {
            /* 路径 A：按实际 tlen 分配并读载荷 */
            dyn = malloc(tlen);
            if (!dyn || esp_partition_read(sp, off + 16 + flen + 16,
                                           dyn, tlen) != ESP_OK) {
                free(dyn); dyn = NULL;
                tw = 0; th = 0; tlen = 0;
            } else {
                payload = dyn;
            }
        }

        if (!tlen) {
            snprintf(item, sizeof(item), "%s{\"slot\":%d,\"fmt\":1,\"w\":0,\"h\":0,\"len\":0,\"data\":\"\"}",
                     i ? "," : "", i);
            httpd_resp_send_chunk(req, item, strlen(item));
        } else {
            snprintf(item, sizeof(item), "%s{\"slot\":%d,\"fmt\":%d,\"w\":%d,\"h\":%d,\"len\":%lu,\"data\":\"",
                     i ? "," : "", i, fmt, tw, th, (unsigned long)tlen);
            httpd_resp_send_chunk(req, item, strlen(item));
            b64_send_chunked(req, payload, tlen);
            httpd_resp_send_chunk(req, "\"}", 2);
        }
        free(dyn);
    }
    httpd_resp_send_chunk(req, "]}", 2);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

/* POST /api/carousel/delete?slot=N：删除并紧凑前移。 */
static esp_err_t carousel_delete_handler(httpd_req_t *req)
{
    power_activity();
    char qbuf[40], vs[8];
    if (httpd_req_get_url_query_len(req) <= 0 ||
        httpd_req_get_url_query_len(req) >= (int)sizeof(qbuf) ||
        httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) != ESP_OK ||
        !kv_get(qbuf, "slot", vs, sizeof(vs)))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad query");
    int slot = atoi(vs);
    if (store_delete_slot(slot) != 0)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad slot");
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "OK");
}

/* GET /api/carousel/full?slot=N：返回整帧二进制（16B 头 + 2bpp 载荷），
   供网页「点击缩略图 → 大图预览」。流式读 flash 分块发送：
   不整帧 malloc（120KB 在堆紧张时失败导致 500），只开 4KB 小块缓冲。 */
static esp_err_t carousel_full_handler(httpd_req_t *req)
{
    power_activity();
    char qbuf[40], vs[8];
    if (httpd_req_get_url_query_len(req) <= 0 ||
        httpd_req_get_url_query_len(req) >= (int)sizeof(qbuf) ||
        httpd_req_get_url_query_str(req, qbuf, sizeof(qbuf)) != ESP_OK ||
        !kv_get(qbuf, "slot", vs, sizeof(vs)))
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad query");
    int slot = atoi(vs);
    if (slot < 0 || slot >= STORE_SLOT_MAX)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad slot");

    const esp_partition_t *sp = store_part();
    if (!sp)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "store off");
    uint32_t off = (uint32_t)slot * STORE_SLOT_SZ;

    uint8_t hdr[16];
    if (esp_partition_read(sp, off, hdr, sizeof(hdr)) != ESP_OK ||
        hdr[0] != STORE_MAGIC0 || hdr[1] != STORE_MAGIC1)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad slot");
    uint32_t len = ((uint32_t)hdr[8] << 24) | ((uint32_t)hdr[9] << 16) |
                   ((uint32_t)hdr[10] << 8) | hdr[11];
    if (len == 0 || len > STORE_FRAME_MAX)
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad slot");

    httpd_resp_set_type(req, "application/octet-stream");
    if (httpd_resp_send_chunk(req, (const char *)hdr, sizeof(hdr)) != ESP_OK)
        return ESP_FAIL;

    uint8_t *chunk = malloc(4096);
    if (!chunk)
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
    uint32_t sent = 0;
    while (sent < len) {
        uint32_t k = (len - sent > 4096) ? 4096 : len - sent;
        if (esp_partition_read(sp, off + 16 + sent, chunk, k) != ESP_OK) {
            free(chunk);
            return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "read fail");
        }
        if (httpd_resp_send_chunk(req, (const char *)chunk, k) != ESP_OK) {
            free(chunk);
            return ESP_FAIL;
        }
        sent += k;
    }
    httpd_resp_send_chunk(req, NULL, 0);
    free(chunk);
    return ESP_OK;
}

/* R1.2.2：改 GPIO 后由页面调用 —— SPI 总线与 ADC 通道只能在启动时按引脚
   初始化，因此引脚生效的唯一途径是重启。先应答再重启，避免页面收不到 200。 */
static esp_err_t reboot_handler(httpd_req_t *req)
{
    power_activity();
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK rebooting");
    ESP_LOGW(TAG, "reboot requested by page");
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_restart();
    return ESP_OK;
}

/* ---------------- CORS 预检（离线客户端走 file://） ---------------- */

static esp_err_t options_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Headers", "Content-Type");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

/* ---------------- HTTP 服务器 ---------------- */

static httpd_handle_t start_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    /* 默认是精确匹配，OPTIONS * 通配路由永远不触发 → 跨源预检 404 →
       离线/本地页面的上传被浏览器拦截。启用通配匹配后预检正常。 */
    cfg.uri_match_fn = httpd_uri_match_wildcard;
    cfg.stack_size = HTTPD_STACK;
    cfg.max_uri_handlers = HTTPD_MAX_URI;
    /* iOS 会并发发起多条 captive 探测连接；容量与 Espressif 官方 captive portal
       示例保持一致，避免低上限下探测连接被 LRU 提前清掉。 */
    cfg.max_open_sockets = 13;
    cfg.lru_purge_enable = true;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed");
        return NULL;
    }

    httpd_uri_t routes[] = {
        { .uri = "/",               .method = HTTP_GET,    .handler = page_handler,         .user_ctx = NULL },
        { .uri = "/api/info",       .method = HTTP_GET,    .handler = info_handler,         .user_ctx = NULL },
        { .uri = "/api/settings",   .method = HTTP_GET,    .handler = settings_get_handler, .user_ctx = NULL },
        { .uri = "/api/settings",   .method = HTTP_POST,   .handler = settings_post_handler,.user_ctx = NULL },
        { .uri = "/api/frame",      .method = HTTP_POST,   .handler = frame_upload_handler, .user_ctx = NULL },
        { .uri = "/api/carousel/clear", .method = HTTP_POST, .handler = clear_carousel_handler, .user_ctx = NULL },
        { .uri = "/api/carousel/list",  .method = HTTP_GET,  .handler = carousel_list_handler,  .user_ctx = NULL },
        { .uri = "/api/carousel/delete",.method = HTTP_POST, .handler = carousel_delete_handler,.user_ctx = NULL },
        { .uri = "/api/carousel/full",  .method = HTTP_GET,  .handler = carousel_full_handler,  .user_ctx = NULL },
        { .uri = "/api/clear",      .method = HTTP_POST,   .handler = clear_handler,        .user_ctx = NULL },
        { .uri = "/api/reboot",     .method = HTTP_POST,   .handler = reboot_handler,       .user_ctx = NULL },
        { .uri = "/api/factory",    .method = HTTP_POST,   .handler = factory_handler,      .user_ctx = NULL },
        { .uri = "/hotspot-detect.html", .method = HTTP_GET, .handler = captive_apple_landing, .user_ctx = NULL },
        { .uri = "/library/test/success.html", .method = HTTP_GET, .handler = captive_apple_landing, .user_ctx = NULL },
        { .uri = "*",               .method = HTTP_OPTIONS,.handler = options_handler,      .user_ctx = NULL },
    };
    for (size_t i = 0; i < sizeof(routes) / sizeof(routes[0]); i++) {
        if (httpd_register_uri_handler(server, &routes[i]) != ESP_OK) {
            ESP_LOGE(TAG, "register %s %d failed", routes[i].uri, routes[i].method);
        }
    }

    ota_web_register(server);

    /* 必须在所有业务路由之后注册：OS 探测路径此前直接 404，根本进不了首页。
       兜底 GET 同时覆盖带查询参数的探测与厂商自定义路径。 */
    const httpd_uri_t captive = {
        .uri = "/*", .method = HTTP_GET, .handler = captive_redirect, .user_ctx = NULL,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &captive));
    return server;
}

/* ---------------- 任务 ---------------- */

static void frame_task(void *arg)
{
    (void)arg;
    for (;;) {
        frame_wait();
        if (frame_display_now() == 0) {
            power_activity();
        }
    }
}

static void idle_monitor_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        if (power_should_sleep()) {
            power_enter_deep_sleep();   /* 不返回 */
        }
    }
}

/* R1.3.0 轮播：切到下一张（读 store 槽并整屏显示）。count<=1 时无动作。
   切图后刷新活动计时，避免刚切完就被空闲深睡。 */
static void carousel_next(void)
{
    int count = 0, cur = 0;
    store_meta_get(&count, &cur);
    if (count <= 0) return;
    cur = (cur + 1) % count;
    store_meta_set(count, cur);
    ESP_LOGI(TAG, "carousel: next -> slot %d (count %d)", cur, count);
    if (frame_display_slot(cur) == 0) {
        power_activity();
    }
}

static void button_task(void *arg)
{
    (void)arg;
    const gpio_num_t btn = (gpio_num_t)settings_pins()->v[PIN_IDX_WAKE];
    int held = 0;

    /* 唤醒语义（R1.3.0）：GPIO 唤醒 = 用户按过键。若此刻键仍按住，等它松开，
       那次松开算一次短按；若已松开（启动慢于松手），补一次切图。 */
    bool pending = s_woken_by_btn;
    bool waiting = pending && (gpio_get_level(btn) == 0);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(50));
        bool down = (gpio_get_level(btn) == 0);
        if (down) {
            held++;
            if (held == HOLD_RESET_S * 20) {
                ESP_LOGW(TAG, "wake button held, keep holding %ds to factory reset", HOLD_RESET_S);
            }
            if (held >= HOLD_RESET_S * 20 + 5) {
                ESP_LOGW(TAG, "long press: factory reset + reboot");
                settings_factory_reset();
                vTaskDelay(pdMS_TO_TICKS(300));
                esp_restart();
            }
        } else {
            if (held > 0) {
                /* 刚松开：<5s 视为短按（含唤醒那次），切下一张 */
                if (held < HOLD_RESET_S * 20) {
                    if (waiting) waiting = false;
                    carousel_next();
                }
                held = 0;
            } else if (pending && !waiting) {
                /* 唤醒瞬间键已松开：补一次切图 */
                pending = false;
                carousel_next();
            }
        }
    }
}

/* ---------------- 主入口 ---------------- */

void app_main(void)
{
    /* ★ MOS 一键开机自锁（R1.2.2）：上电第一件事就拉高，按钮松手后靠它维持
       供电。必须早于一切耗时初始化，也不依赖任何设置（引脚固定 GPIO2）。 */
    gpio_config_t self_lock = {
        .pin_bit_mask = 1ULL << PIN_SELF_LOCK,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&self_lock);
    gpio_set_level((gpio_num_t)PIN_SELF_LOCK, 1);

    ESP_LOGI(TAG, "=== MoInk %s | 4-color | ap + unified upgrade ===", MOINK_VERSION);

    /* NVS */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    settings_init();

    /* 唤醒原因：GPIO = 按键，TIMER = 定时自动唤醒。放在 settings_init() 之后，
       日志才能打出运行时解析出的唤醒脚。 */
    uint32_t wake = power_wakeup_causes();
    bool auto_wake = (wake & (1u << ESP_SLEEP_WAKEUP_TIMER)) != 0;
    power_set_auto_wake(auto_wake);
    if (wake & (1u << ESP_SLEEP_WAKEUP_GPIO)) {
        ESP_LOGI(TAG, "woken by GPIO%u button", settings_pins()->v[PIN_IDX_WAKE]);
        s_woken_by_btn = true;   /* R1.3.0：唤醒的这次按下也算一次短按（切图） */
    } else if (auto_wake) {
        ESP_LOGI(TAG, "woken by RTC timer (auto wake)");
    } else if (wake) {
        ESP_LOGI(TAG, "wakeup bitmap 0x%lx", (unsigned long)wake);
    }

    power_init();
    store_init();       /* 轮播存储底座（FB-017 第一部分）：只探测，不写入 */

    if (epd_init() != 0) {
        ESP_LOGE(TAG, "EPD init failed");
    }
    epd_set_panel((epd_panel_t)settings_get()->panel);
    epd_set_hflip(settings_get()->hflip != 0);

    frame_init();
    netif_ap_init();
    captive_dns_start();

    if (start_server() == NULL) {
        ESP_LOGE(TAG, "HTTP server failed to start");
    }

    xTaskCreate(frame_task, "frame", 4096, NULL, 5, NULL);
    xTaskCreate(idle_monitor_task, "idle_mon", 3072, NULL, 4, NULL);
    xTaskCreate(button_task, "button", 3072, NULL, 4, NULL);

    ota_web_confirm();

    ESP_LOGI(TAG, "ready: %s @ http://192.168.4.1 (heap %lu)",
             netif_ap_ssid(), (unsigned long)esp_get_free_heap_size());
}
