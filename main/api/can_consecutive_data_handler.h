#ifndef AMICE_CAN_CONSECUTIVE_DATA_HANDLER_H
#define AMICE_CAN_CONSECUTIVE_DATA_HANDLER_H


#include "api_context.h"

#include <http_server/server.h>

namespace api {

template<typename InfoMatcher, uint32_t Start, uint32_t End>
struct CANConsecutiveDataHandler {
  static void Handle(Context* context, httpd_req_t* req) {
    if (InfoMatcher::Match(context->bus_status().bus_info())) {
      std::array<uint64_t, End-Start+1> raw_data{};
      for (size_t i = Start; i <= End; i++) {
        const auto frame = context->store().frame(i);
        if (frame) {
          raw_data[i-Start] = (*frame).data;
        } else {
          raw_data[i-Start] = 0;
        }
      }

      httpd_resp_set_type(req, "application/octet-stream");
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      httpd_resp_send(req, reinterpret_cast<const char*>(raw_data.data()), sizeof(raw_data));
    } else {
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
      httpd_resp_set_status(req, "400 Bad Request");
      httpd_resp_send(req, "", 0);
    }
  }

  static constexpr const httpd_method_t Method = HTTP_GET;
};

}

#endif //AMICE_CAN_CONSECUTIVE_DATA_HANDLER_H
