#ifndef __STORAGE_H
#define __STORAGE_H

#include <sys/types.h>

enum nvs_id {
    TMC9660_PARAMS_STORAGE = 0,
};

void storage_init();
int storage_get(enum nvs_id id, void *data, size_t len);
int storage_put(enum nvs_id id, void *data, size_t len);

#endif /* __STORAGE_H */
