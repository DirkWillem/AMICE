#ifndef AMICE_API_SERVER_H
#define AMICE_API_SERVER_H

#include <http_server/server.h>

#include "api_context.h"
#include "ping_handler.h"
#include "bus_status_handler.h"

#include "can_data/era_can_data.h"

namespace api {

using ApiServer = http::Server<Context,
    PingHandler,

    BusStatusHandler,

    EraGlobalInfoHandler,

    EraBatteryInfoHandler,
    EraBatteryStateHandler,
    EraBatteryCellDataHandler>;

}

#endif //AMICE_API_SERVER_H
