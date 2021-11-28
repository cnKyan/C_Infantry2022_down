//
// Created by HLiamso on 2021-11-15.
//

#ifndef DETECT_H
#define DETECT_H

#endif //DETECT_H
#include "stdint.h"
#include "typedefine.h"
void global_err_detector_init(void);
void err_detector_hook(int err_id);
void detect_task(const void* argu);

void module_offline_callback(void);

extern glb_err_type_t glb_err;