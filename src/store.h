#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "esp_partition.h"

/* 槽内帧头魔数（与 frame.h 的 FRAME_MAGIC 一致：0xA5 0x5A）——store 分区里
   存的就是前端上传的帧头，校验魔数即校验槽有效性。 */
#define STORE_MAGIC0 0xA5
#define STORE_MAGIC1 0x5A

/*
 * 轮播帧存储（FB-017 第二部分，随 R1.3.0 并入；R1.3.1 交互定案调整）：
 * 在第一部分（分区探测 + 容量上报）之上，实现真正的帧存取与轮播元数据。
 *
 * 布局（延续第一部分的约定）：
 *   - 槽位固定 128 KB（STORE_SLOT_SZ），共 STORE_SLOT_MAX 个；
 *   - 每槽存「16 字节帧头 + 2bpp 载荷」（STORE_ENTRY_MAX = 16 + 120000），
 *     768x552 几何时整条 = 16 + 105984 = 106000 B，一槽一张；
 *
 * 写入侧（R1.3.1 起为「追加到末尾」）：store_save_frame() 整槽擦写 + 顺序写头与
 *   载荷，新图永远写到 count 槽（0..count-1 为有效图），成功后 count++、cur 指向
 *   新图；轮播满 8 张时拒绝上传（返回 STORE_ERR_FULL），不再轮转覆盖。
 * 读取侧：store_load_slot() 读出「头 + 载荷」，由 frame.c 校验魔数后交给屏驱动。
 * 删除侧：store_delete_slot() 删除并紧凑——后续槽整体前移一位（保持顺序连续），
 *   count--、cur 修正，新图继续追加到新末尾。
 * 缩略侧：store_thumb_frame() 4x4 抽稀取众数色 → 192x138 小帧（16B 头 + 6624B 载荷），
 *   位序与整帧同一契约，供网页图库列表渲染。
 * 并发：读写共用一把互斥锁（上传写槽与按键切图读槽互斥）。
 */

#define STORE_SLOT_SZ   0x20000u   /* 单槽 128 KB */
#define STORE_SLOT_MAX  8u         /* 0x110000 / 0x20000 */
#define STORE_FRAME_MAX 120000u    /* 一帧的最大字节数（800x600 2bpp，历史上限） */
#define STORE_ENTRY_MAX (16u + STORE_FRAME_MAX)   /* 头 + 载荷的上限 */

/* 轮播缩略图（R1.3.2 起随帧预存，槽尾余量 ~24.5KB）：
   - fmt=1：2bit 缩略帧（16B 头 + 载荷，4x4 抽稀 192x138→6624B；老图回退现算）
   - fmt=2：彩色 JPEG 缩略（R1.3.3，网页端从 RGB 合成图生成，竖向 138x192，
     质量 0.72，~3-8KB），仅手机端图库显示，不经过墨水屏 */
#define STORE_THUMB_W   200u
#define STORE_THUMB_H   150u
#define STORE_THUMB_LEN (STORE_THUMB_W / 4u * STORE_THUMB_H)      /* 2bit 载荷上限 7500 */
#define STORE_THUMB_ENTRY (16u + STORE_THUMB_LEN)                 /* 7516 */
#define STORE_THUMB_JPEG_MAX 20000u   /* 彩色 JPEG 缩略上限（槽余量内） */
#define STORE_THUMB_ENTRY_MAX (16u + STORE_THUMB_JPEG_MAX)        /* 20016 */

/* 保存失败返回值：-1 = 通用失败；-2 = 轮播已满（STORE_SLOT_MAX），需删除/清空。 */
#define STORE_ERR_FULL  (-2)

/* 启动时探测 "store" 分区；缺失（旧分区表 OTA 上来的设备）时 slots = 0。 */
void store_init(void);

/* 可用槽位数；0 = 分区不存在，轮播功能不可用。 */
int store_slots(void);

/* 返回 store 分区句柄（供 main.c 的 /api/carousel/full 流式读取；分区未初始化
   返回 NULL）。 */
const esp_partition_t *store_part(void);

/*
 * 轮播元数据：
 *   count = 有效张数（0..STORE_SLOT_MAX）
 *   cur   = 当前显示槽（0..count-1；count==0 时无意义）
 * 切图 = cur 在 [0, count-1] 内循环（写入从槽 0 起连续，槽号即逻辑序号）。
 */
void store_meta_get(int *count, int *cur);
void store_meta_set(int count, int cur);

/*
 * 保存一帧：整槽擦写 + 顺序写 hdr（帧头 16B）与 payload（2bpp 载荷），
 * 若 thumb 非 NULL 则在帧后追加「缩略帧头 16B + 缩略载荷 tlen」（R1.3.2，
 * 离线预生成缩略图随帧上传）。新图写到末尾槽（= 当前 count），成功后
 * count++ / cur 指向新图。轮播满（count == STORE_SLOT_MAX）时返回
 * STORE_ERR_FULL，不覆盖已有图。成功返回写入的槽号，通用失败 < 0。
 * 注意：hdr 必须为 16 字节帧头（与 frame.c 的 FRAME_HDR_LEN 契约一致）；
 * thumb 指向「16B 缩略头 + tlen 载荷」，tlen 为缩略载荷字节数。
 */
int store_save_frame(const uint8_t *hdr, uint32_t hdr_len,
                     const uint8_t *payload, uint32_t len,
                     const uint8_t *thumb, uint32_t tlen);

/*
 * 读出槽 slot：填充 buf（前 16 字节为帧头，其后为载荷）。
 * 成功返回总字节数（>0）；空槽 / 魔数不符 / 载荷超上限返回 < 0。
 * cap 必须 >= 16 + 载荷长（768x552 帧时 >= 106000；调用方用 EPD_MAX_BUF_LEN）。
 */
int store_load_slot(int slot, uint8_t *buf, uint32_t cap);

/* 删除槽 slot 并紧凑：slot+1..count-1 依次前移一位，count--，当前槽号随之
   修正（新图继续追加到新末尾）。成功返回 0；slot 越界/空返回 -1。 */
int store_delete_slot(int slot);

/*
 * 读出槽 slot 的预存缩略帧（R1.3.2：网页离线预生成，存于帧后槽尾）：
 * 填充 out（16B 缩略头 + 载荷；头[2] = 1 为 2bit 缩略，2 为彩色 JPEG）。
 * 成功返回总字节数（=16+tlen）；无缩略 / 魔数不符 / 超上限返回 -1
 * （调用方回退 store_thumb_frame 现算）。cap 必须 >= STORE_THUMB_ENTRY_MAX。
 */
int store_load_thumb(int slot, uint8_t *out, uint32_t cap);

/*
 * 生成槽 slot 的缩略小帧（16 字节帧头 + 载荷，4x4 抽稀取众数色，位序与
 * 整帧同一契约；无预存缩略时的回退路径）。成功返回 16+tlen；
 * 槽空 / 几何非法 / 缓冲不足返回 -1。cap 必须 >= STORE_THUMB_ENTRY。
 */
int store_thumb_frame(int slot, uint8_t *out, uint32_t cap);

/* 清空轮播：只清元数据（count=0 / cur=0 / 写指针=0），不擦 flash ——
   槽内容留待下次写入整槽覆盖，省一次全分区擦写。 */
void store_clear(void);
