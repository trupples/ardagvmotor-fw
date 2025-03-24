#include "storage.h"
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

#define NVS_PARTITION        storage_partition
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)

struct nvs_fs fs = {
    .flash_device = NVS_PARTITION_DEVICE,
    .offset = NVS_PARTITION_OFFSET,
    .sector_size = 8192, // Page size = 8192
    .sector_count = 2, // Fill a page
};

void storage_init() {
    int ret = nvs_mount(&fs);

    if(ret) {
        LOG_ERR("Could not mount NVS");
    }

    LOG_INF("NVS free space: %d", nvs_calc_free_space(&fs));
}

int storage_get(enum nvs_id id, void *data, size_t len) {
    int ret = nvs_read(&fs, id, data, len);
    if(ret < 0) return ret;
    return 0;
}

int storage_put(enum nvs_id id, void *data, size_t len) {
    int ret = nvs_write(&fs, id, data, len);
    if(ret < 0) return ret;
    return 0;
}
