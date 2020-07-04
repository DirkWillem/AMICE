#ifndef AMICE_PING_HANDLER_H
#define AMICE_PING_HANDLER_H

#include <http_server/server.h>

#include "api_context.h"

namespace api {

/**
 * Handler for a Ping call
 */
struct PingHandler {
    static void Handle(Context* context, httpd_req_t* req);

    static constexpr const char* URI = "/ping";
    static constexpr const httpd_method_t Method = HTTP_GET;
};

}

#endif //AMICE_PING_HANDLER_H
