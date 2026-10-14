/*
 * 墨印 · MoInk — store 原始帧存储（R1.2.0 底座 + R1.3.0 读写 + R1.3.1 交互定案）
 *
 * 职责一句话：把「帧头 + 2bpp 载荷」整槽写进 store 分区，支持按键轮播。
 *
 * 为什么不用文件系统（LittleFS / FATFS）：轮播帧是「定长大对象、写一次读多次」，
 * 整槽写入的写放大 = 恰好 1（每张图消耗一次整槽擦除），没有任何文件系统
 * 元数据开销；LittleFS 的写时复制与元数据提交反而会带来额外擦写。
 *
 * R1.3.0（FB-017 第二部分）：实现帧存取 + 轮播元数据（NVS "store" 命名空间），
 *   写入为 round-robin 轮转覆盖。
 * R1.3.1（交互定案）：语义改为「追加到末尾」——
 *   - 新图永远写到 count 槽，轮播满 8 张时拒绝上传（STORE_ERR_FULL），不再轮转覆盖；
 *   - store_delete_slot() 删除后把后续槽整体前移一位（顺序紧凑），新图继续追加；
 *   - store_thumb_frame() 输出 96x69 的 8x8 抽稀缩略小帧，供网页缩略图列表渲染。
 *   读取：读槽 -> 校验魔数（0xA5 0x5A）-> 按头里的载荷长读回 -> 交 frame.c 显示。
 *   清空：只清 NVS 元数据，不擦 flash（下次写入整槽覆盖，省一次全分区擦写）。
 */
#include "store.h"

#include <string.h>

#include "esp_log.h"
#include "esp_partition.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static const char *TAG = "store";

#define NVS_NS      "store"
#define KEY_COUNT   "cnt"
#define KEY_CUR     "cur"
#define KEY_WIDX    "widx"

/* 帧头魔数（与 frame.h 的 FRAME_MAGIC0/1 一致，此处按值校验避免头文件耦合）。 */
/* 槽内帧头魔数见 store.h（STORE_MAGIC0/1）。 */static const esp_partition_t *s_part;
static uint32_t s_slots;
static SemaphoreHandle_t s_lock;

/* ---------------- NVS 小工具 ---------------- */

static int st_nvs_get_u8(const char *k, uint8_t *v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READONLY, &h) != ESP_OK) return -1;
    esp_err_t e = nvs_get_u8(h, k, v);
    nvs_close(h);
    if (e == ESP_ERR_NVS_NOT_FOUND) { *v = 0; return 0; }
    return (e == ESP_OK) ? 0 : -1;
}

static int st_nvs_set_u8(const char *k, uint8_t v)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS, NVS_READWRITE, &h) != ESP_OK) return -1;
    esp_err_t e = nvs_set_u8(h, k, v);
    if (e == ESP_OK) e = nvs_commit(h);
    nvs_close(h);
    return (e == ESP_OK) ? 0 : -1;
}

/* ---------------- 初始化 ---------------- */

void store_init(void)
{
    s_part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
                                      ESP_PARTITION_SUBTYPE_ANY, "store");
    if (!s_part) {
        s_slots = 0;
        ESP_LOGW(TAG, "store partition not found (flashed over an older "
                      "partition table via OTA?) - carousel storage unavailable");
        return;
    }

    s_slots = s_part->size / STORE_SLOT_SZ;
    if (s_slots > STORE_SLOT_MAX) s_slots = STORE_SLOT_MAX;
    s_lock = xSemaphoreCreateMutex();

    /* 元数据归一：越界值回落，避免历史脏数据把轮播指针带飞。 */
    uint8_t c = 0, w = 0;
    st_nvs_get_u8(KEY_COUNT, &c);
    st_nvs_get_u8(KEY_WIDX, &w);
    if (c > STORE_SLOT_MAX) { c = 0; st_nvs_set_u8(KEY_COUNT, c); }
    if (w >= STORE_SLOT_MAX) { w = 0; st_nvs_set_u8(KEY_WIDX, w); }

    ESP_LOGI(TAG, "store: off=0x%06lx size=%lu KB, %lu slot(s) x %u KB, "
                  "count=%u widx=%u",
             (unsigned long)s_part->address, (unsigned long)(s_part->size / 1024),
             (unsigned long)s_slots, (unsigned)(STORE_SLOT_SZ / 1024),
             c, w);
}

int store_slots(void)
{
    return (int)s_slots;
}

const esp_partition_t *store_part(void)
{
    return s_part;
}

/* ---------------- 元数据 ---------------- */

void store_meta_get(int *count, int *cur)
{
    uint8_t c = 0, k = 0;
    st_nvs_get_u8(KEY_COUNT, &c);
    st_nvs_get_u8(KEY_CUR, &k);
    if (c > STORE_SLOT_MAX) c = 0;
    if (k >= c) k = 0;
    if (count) *count = (int)c;
    if (cur)   *cur = (int)k;
}

void store_meta_set(int count, int cur)
{
    if (count < 0 || count > STORE_SLOT_MAX) return;
    if (cur < 0 || cur >= count) cur = 0;
    st_nvs_set_u8(KEY_COUNT, (uint8_t)count);
    st_nvs_set_u8(KEY_CUR, (uint8_t)cur);
}

/* ---------------- 读写 ---------------- */

/* CRC-16/CCITT-FALSE（与 frame.c 相同契约：poly 0x1021, init 0xFFFF）。 */
static uint16_t st_crc16(const uint8_t *d, size_t n)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

/* 保存一帧：新图追加到末尾槽（= 当前 count）。轮播满 8 时拒绝（STORE_ERR_FULL），
   不再轮转覆盖。成功后 count++、cur 指向新图，全部落 NVS。 */
int store_save_frame(const uint8_t *hdr, uint32_t hdr_len,
                     const uint8_t *payload, uint32_t len,
                     const uint8_t *thumb, uint32_t tlen)
{
    if (!s_part || !s_lock) return -1;
    if (hdr_len != 16) return -1;
    if (len == 0 || len > STORE_FRAME_MAX) return -1;
    uint32_t thumb_extra = (thumb && tlen > 0 && tlen <= STORE_THUMB_JPEG_MAX)
                           ? 16u + tlen : 0u;
    if (hdr_len + len + thumb_extra > STORE_SLOT_SZ) return -1;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return -1;

    uint8_t cnt = 0;
    st_nvs_get_u8(KEY_COUNT, &cnt);
    if (cnt > STORE_SLOT_MAX) cnt = 0;
    if (cnt >= STORE_SLOT_MAX) {
        xSemaphoreGive(s_lock);
        return STORE_ERR_FULL;      /* 已满：请先删除或清空 */
    }

    int slot = (int)cnt;            /* 追加到末尾 */
    uint32_t off = (uint32_t)slot * STORE_SLOT_SZ;

    esp_err_t e = esp_partition_erase_range(s_part, off, STORE_SLOT_SZ);
    if (e == ESP_OK) e = esp_partition_write(s_part, off, hdr, hdr_len);
    if (e == ESP_OK) e = esp_partition_write(s_part, off + hdr_len, payload, len);
    if (e == ESP_OK && thumb_extra)
        e = esp_partition_write(s_part, off + hdr_len + len, thumb,
                                thumb_extra);    /* 缩略头+缩略载荷整体 */

    if (e != ESP_OK) {
        ESP_LOGE(TAG, "slot %d write failed: %s", slot, esp_err_to_name(e));
        xSemaphoreGive(s_lock);
        return -1;
    }

    st_nvs_set_u8(KEY_COUNT, cnt + 1);
    st_nvs_set_u8(KEY_CUR, (uint8_t)slot);

    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "saved frame -> slot %d (count=%u)", slot, cnt + 1);
    return slot;
}

/* 删除槽 slot 并紧凑：slot+1..count-1 依次前移一位，count--。
   cur 修正：被删槽在前则 cur--；正好删当前图则 cur 指向前移后的同位置；
   写指针不再单独维护（新图始终追加到 count 槽）。 */
int store_delete_slot(int slot)
{
    if (!s_part || !s_lock) return -1;

    uint8_t cnt = 0, cur = 0;
    st_nvs_get_u8(KEY_COUNT, &cnt);
    st_nvs_get_u8(KEY_CUR, &cur);
    if (cnt > STORE_SLOT_MAX) cnt = 0;
    if ((int)cnt <= 0 || slot < 0 || slot >= (int)cnt) return -1;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return -1;

    uint8_t *buf = malloc(4096);
    if (!buf) {
        xSemaphoreGive(s_lock);
        return -1;
    }

    int err = 0;
    for (int j = slot + 1; j < (int)cnt; j++) {
        uint32_t src = (uint32_t)j * STORE_SLOT_SZ;
        uint32_t dst = (uint32_t)(j - 1) * STORE_SLOT_SZ;
        /* 流式块复制：读帧头拿真实长度，探测缩略区（R1.3.2），按 4KB 块
           整体搬「帧头+帧载荷[+缩略头+缩略载荷]」，不整帧 malloc */
        uint8_t hdr[16];
        if (esp_partition_read(s_part, src, hdr, sizeof(hdr)) != ESP_OK ||
            hdr[0] != STORE_MAGIC0 || hdr[1] != STORE_MAGIC1) { err = -1; break; }
        uint32_t len = ((uint32_t)hdr[8] << 24) | ((uint32_t)hdr[9] << 16) |
                       ((uint32_t)hdr[10] << 8) | hdr[11];
        if (len == 0 || len > STORE_FRAME_MAX) { err = -1; break; }
        uint32_t tlen = 0;
        uint8_t thdr[16];
        if (esp_partition_read(s_part, src + 16 + len, thdr, sizeof(thdr)) == ESP_OK &&
            thdr[0] == STORE_MAGIC0 && thdr[1] == STORE_MAGIC1) {
            uint32_t tl = ((uint32_t)thdr[8] << 24) | ((uint32_t)thdr[9] << 16) |
                          ((uint32_t)thdr[10] << 8) | thdr[11];
            uint32_t lim = (thdr[2] == 2) ? STORE_THUMB_JPEG_MAX
                                           : STORE_THUMB_LEN;
            if (tl && tl <= lim) tlen = tl;
        }
        uint32_t total = 16u + len + (tlen ? 16u + tlen : 0u);
        if (esp_partition_erase_range(s_part, dst, STORE_SLOT_SZ) != ESP_OK) { err = -1; break; }
        uint32_t off = 0;
        while (off < total) {
            uint32_t k = (total - off > 4096) ? 4096 : total - off;
            if (esp_partition_read(s_part, src + off, buf, k) != ESP_OK ||
                esp_partition_write(s_part, dst + off, buf, k) != ESP_OK) { err = -1; break; }
            off += k;
        }
        if (err) break;
    }
    free(buf);

    if (err == 0) {
        uint8_t nc = (uint8_t)(cnt - 1);
        uint8_t ncur = cur;
        if (slot < (int)cur) ncur = (uint8_t)(cur - 1);
        else if (slot == (int)cur)
            ncur = (nc == 0) ? 0 : (uint8_t)((cur >= nc) ? nc - 1 : cur);
        st_nvs_set_u8(KEY_COUNT, nc);
        st_nvs_set_u8(KEY_CUR, ncur);
        ESP_LOGI(TAG, "deleted slot %d -> count=%u cur=%u", slot, nc, ncur);
    }

    xSemaphoreGive(s_lock);
    return err;
}

/* 生成槽 slot 的缩略小帧：4x4 抽稀取众数色，位序与整帧（packIdx）同一契约：
   行自下而上、列右到左、字节内 MSB 优先。输出 16B 帧头 + 抽稀载荷，
   头内写实际几何（768x552→192x138，800x600→200x150）。
   内存友好：不整帧 malloc，按「缩略行」读 4 个源行缓冲（最大 4*200B），
   idx 抽稀图最大 200*150=30000B。每个失败点打 ERROR 日志便于定位。 */
int store_thumb_frame(int slot, uint8_t *out, uint32_t cap)
{
    if (!s_part || !s_lock) return -1;
    if (cap < STORE_THUMB_ENTRY) return -1;
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return -1;

    uint32_t off = (uint32_t)slot * STORE_SLOT_SZ;
    uint8_t hdr[16];
    if (esp_partition_read(s_part, off, hdr, sizeof(hdr)) != ESP_OK) {
        ESP_LOGE(TAG, "thumb: slot %d header read failed", slot);
        xSemaphoreGive(s_lock);
        return -1;
    }
    if (hdr[0] != STORE_MAGIC0 || hdr[1] != STORE_MAGIC1) {
        ESP_LOGE(TAG, "thumb: slot %d bad magic (%02X %02X)",
                 slot, hdr[0], hdr[1]);
        xSemaphoreGive(s_lock);
        return -1;
    }
    uint32_t w = ((uint32_t)hdr[4] << 8) | hdr[5];
    uint32_t h = ((uint32_t)hdr[6] << 8) | hdr[7];
    uint32_t plen = ((uint32_t)hdr[8] << 24) | ((uint32_t)hdr[9] << 16) |
                    ((uint32_t)hdr[10] << 8) | hdr[11];
    /* 4x4 抽稀要求源几何能被 4 整除，且与载荷自洽 */
    if (!w || !h || (w & 3) || (h & 3) || plen != (w >> 2) * h) {
        ESP_LOGE(TAG, "thumb: slot %d bad geom %lux%lu plen=%lu", slot,
                 (unsigned long)w, (unsigned long)h, (unsigned long)plen);
        xSemaphoreGive(s_lock);
        return -1;
    }

    uint32_t tw = w >> 2, th = h >> 2;
    uint32_t tlen = (tw >> 2) * th;
    if (!tlen || tw > STORE_THUMB_W || th > STORE_THUMB_H || 16u + tlen > cap) {
        ESP_LOGE(TAG, "thumb: slot %d size overflow tw=%lu th=%lu tlen=%lu", slot,
                 (unsigned long)tw, (unsigned long)th, (unsigned long)tlen);
        xSemaphoreGive(s_lock);
        return -1;
    }

    uint32_t RB = w >> 2;
    uint8_t *rows = malloc(4u * RB);       /* 单缩略行的 4 个源行缓冲（≤800B） */
    uint8_t *idxp = malloc(tlen);          /* 抽稀索引 2bit/像素打包（≤7500B） */
    if (!rows || !idxp) {
        ESP_LOGE(TAG, "thumb: slot %d malloc fail rows=%lu idxp=%lu", slot,
                 (unsigned long)(4u * RB), (unsigned long)tlen);
        free(rows);
        free(idxp);
        xSemaphoreGive(s_lock);
        return -1;
    }
    memset(idxp, 0, tlen);

    /* 众数抽稀：每缩略行读 4 个源行，逐块统计代表色，写入 idxp 的 2bit 位 */
    for (uint32_t ty = 0; ty < th; ty++) {
        for (uint32_t dy = 0; dy < 4; dy++) {
            uint32_t y = ty * 4 + dy;
            uint32_t rowoff = off + 16 + (h - 1 - y) * RB;   /* 行自下而上契约 */
            if (esp_partition_read(s_part, rowoff, rows + dy * RB, RB) != ESP_OK) {
                ESP_LOGE(TAG, "thumb: slot %d row read failed @y=%lu", slot,
                         (unsigned long)y);
                free(rows);
                free(idxp);
                xSemaphoreGive(s_lock);
                return -1;
            }
        }
        for (uint32_t tx = 0; tx < tw; tx++) {
            uint32_t vote[4] = { 0, 0, 0, 0 };
            for (uint32_t dy = 0; dy < 4; dy++) {
                const uint8_t *row = rows + dy * RB;
                for (uint32_t dx = 0; dx < 4; dx++) {
                    uint32_t x = tx * 4 + dx;
                    uint8_t v = row[(w - 1 - x) >> 2];
                    vote[(v >> ((x & 3) * 2)) & 3]++;
                }
            }
            uint32_t best = 0;
            for (uint32_t k = 1; k < 4; k++)
                if (vote[k] > vote[best]) best = k;
            uint32_t pi = ty * tw + tx;
            idxp[pi >> 2] |= (uint8_t)(best << ((pi & 3) * 2));
        }
    }

    /* 按 packIdx 位序打包缩略帧载荷（行自下而上、列右到左、MSB 优先） */
    uint32_t TRB = tw >> 2;
    uint8_t *tpayload = out + 16;
    for (uint32_t r = 0; r < th; r++) {
        uint32_t srcrow = (th - 1 - r) * tw, bo = r * TRB;
        for (uint32_t xb = 0; xb < TRB; xb++) {
            uint32_t xa = tw - 4 - (xb << 2);
            uint8_t v = 0;
            for (uint32_t k = 0; k < 4; k++) {
                uint32_t pi = srcrow + xa + k;
                v |= (uint8_t)(((idxp[pi >> 2] >> ((pi & 3) * 2)) & 3) << (k * 2));
            }
            tpayload[bo + xb] = v;
        }
    }

    uint16_t crc = st_crc16(tpayload, tlen);
    out[0] = STORE_MAGIC0; out[1] = STORE_MAGIC1;
    out[2] = hdr[2];                        /* 帧格式版本沿用整帧 */
    out[3] = hdr[3];                        /* 面板类型沿用 */
    out[4] = (uint8_t)(tw >> 8); out[5] = (uint8_t)tw;
    out[6] = (uint8_t)(th >> 8); out[7] = (uint8_t)th;
    out[8]  = (uint8_t)(tlen >> 24); out[9]  = (uint8_t)(tlen >> 16);
    out[10] = (uint8_t)(tlen >> 8); out[11] = (uint8_t)tlen;
    out[12] = (uint8_t)(crc >> 8); out[13] = (uint8_t)crc;
    out[14] = 0; out[15] = 0;

    free(rows);
    free(idxp);
    xSemaphoreGive(s_lock);
    return (int)(16u + tlen);
}

int store_load_slot(int slot, uint8_t *buf, uint32_t cap)
{
    if (!s_part || !s_lock) return -1;
    if (slot < 0 || slot >= (int)s_slots) return -1;
    if (cap < 16) return -1;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return -1;

    uint32_t off = (uint32_t)slot * STORE_SLOT_SZ;
    uint8_t hdr[16];
    esp_err_t e = esp_partition_read(s_part, off, hdr, sizeof(hdr));
    int total = -1;

    if (e == ESP_OK && hdr[0] == STORE_MAGIC0 && hdr[1] == STORE_MAGIC1) {
        uint32_t len = ((uint32_t)hdr[8] << 24) | ((uint32_t)hdr[9] << 16) |
                       ((uint32_t)hdr[10] << 8) | hdr[11];
        if (len && len <= STORE_FRAME_MAX && 16u + len <= cap) {
            e = esp_partition_read(s_part, off + 16, buf + 16, len);
            if (e == ESP_OK) {
                memcpy(buf, hdr, 16);
                total = 16 + (int)len;
            }
        }
    }

    xSemaphoreGive(s_lock);
    if (total < 0) ESP_LOGW(TAG, "slot %d empty or bad (magic/len)", slot);
    return total;
}

/* 读出槽 slot 的预存缩略帧（R1.3.2：网页离线预生成，存于帧后槽尾）。
   布局：帧头(16) + 帧载荷(flen) + 缩略头(16) + 缩略载荷(tlen)。
   缩略头[2]：1=2bit 缩略（上限 STORE_THUMB_LEN），2=彩色 JPEG（上限
   STORE_THUMB_JPEG_MAX）。 */
int store_load_thumb(int slot, uint8_t *out, uint32_t cap)
{
    if (!s_part || !s_lock) return -1;
    if (slot < 0 || slot >= (int)s_slots) return -1;
    if (cap < STORE_THUMB_ENTRY_MAX) return -1;

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(5000)) != pdTRUE) return -1;

    uint32_t off = (uint32_t)slot * STORE_SLOT_SZ;
    uint8_t fhdr[16], thdr[16];
    int total = -1;
    if (esp_partition_read(s_part, off, fhdr, sizeof(fhdr)) == ESP_OK &&
        fhdr[0] == STORE_MAGIC0 && fhdr[1] == STORE_MAGIC1) {
        uint32_t flen = ((uint32_t)fhdr[8] << 24) | ((uint32_t)fhdr[9] << 16) |
                        ((uint32_t)fhdr[10] << 8) | fhdr[11];
        if (flen && flen <= STORE_FRAME_MAX &&
            esp_partition_read(s_part, off + 16 + flen, thdr,
                               sizeof(thdr)) == ESP_OK &&
            thdr[0] == STORE_MAGIC0 && thdr[1] == STORE_MAGIC1) {
            uint32_t tlen = ((uint32_t)thdr[8] << 24) | ((uint32_t)thdr[9] << 16) |
                            ((uint32_t)thdr[10] << 8) | thdr[11];
            uint32_t lim = (thdr[2] == 2) ? STORE_THUMB_JPEG_MAX
                                           : STORE_THUMB_LEN;
            if (tlen && tlen <= lim && 16u + tlen <= cap &&
                esp_partition_read(s_part, off + 16 + flen + 16, out + 16,
                                   tlen) == ESP_OK) {
                memcpy(out, thdr, 16);
                total = 16 + (int)tlen;
            }
        }
    }

    xSemaphoreGive(s_lock);
    if (total < 0) ESP_LOGW(TAG, "slot %d no pre-stored thumb", slot);
    return total;
}

void store_clear(void)
{
    st_nvs_set_u8(KEY_COUNT, 0);
    st_nvs_set_u8(KEY_CUR, 0);
    st_nvs_set_u8(KEY_WIDX, 0);
    ESP_LOGI(TAG, "carousel cleared (metadata only, slots left for rotation)");
}
