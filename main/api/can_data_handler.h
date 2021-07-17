#ifndef AMICE_CAN_DATA_HANDLER_H
#define AMICE_CAN_DATA_HANDLER_H

#include "api_context.h"

#include <http_server/server.h>

namespace api {

template<typename... Msgs>
struct CANMessageList {};

template<typename Matcher, typename MsgList>
struct CANDataHandler;

template<typename InfoMatcher, typename... Msgs>
struct CANDataHandler<InfoMatcher, CANMessageList<Msgs...>> {
    static void Handle(Context* context, httpd_req_t* req) {
        if (InfoMatcher::Match(context->bus_status().bus_info())) {
            std::array<can::Frame, sizeof...(Msgs)> buffer{};
            context->store().ToBuffer<Msgs::ID...>(buffer);

            httpd_resp_set_type(req, "application/octet-stream");
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_send(req, reinterpret_cast<const char*>(buffer.data()), sizeof(buffer));
        } else {
            httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
            httpd_resp_set_status(req, "400 Bad Request");
            httpd_resp_send(req, "", 0);
        }
    }

    static constexpr const httpd_method_t Method = HTTP_GET;
};

}


#endif //AMICE_CAN_DATA_HANDLER_H
