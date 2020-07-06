#ifndef AMICE_BUS_STATUS_HANDLER_H
#define AMICE_BUS_STATUS_HANDLER_H

#include <http_server/server.h>

#include "api_context.h"

namespace api {

/**
 * Handler for a Bus status call
 */
struct BusStatusHandler {
    static void Handle(Context* context, httpd_req_t* req);

    static constexpr const char* URI = "/bus/status";
    static constexpr const httpd_method_t Method = HTTP_GET;
};

}

#endif //AMICE_BUS_STATUS_HANDLER_H
