#include "bus_status_handler.h"

#include <cJSON.h>


namespace api {

void BusStatusHandler::Handle(Context *context, httpd_req_t *req) {
    char buf[4096]{};

    auto obj = cJSON_CreateObject();
    cJSON_AddNumberToObject(obj, "msgFrequency", context->bus_status().message_frequency());
    cJSON_AddNumberToObject(obj, "busLoad", context->bus_status().bus_load());

    auto info = context->bus_status().bus_info();
    auto system = cJSON_CreateObject();
    cJSON_AddBoolToObject(system, "isEra", info.is_stella_era);
    cJSON_AddItemToObject(obj, "system", system);

    cJSON_PrintPreallocated(obj, buf, 4096, cJSON_False);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "application/json");
    httpd_resp_send(req, buf, strlen(buf));
}

}