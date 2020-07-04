#include "ping_handler.h"

#include <cJSON.h>

namespace api {

void PingHandler::Handle(Context* context, httpd_req_t* req) {
    char buf[4096]{};

    auto obj = cJSON_CreateObject();
    cJSON_AddStringToObject(obj, "ping", "pong");

    cJSON_PrintPreallocated(obj, buf, 4096, cJSON_False);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Content-Type", "application/json");
    httpd_resp_send(req, buf, strlen(buf));
}

}