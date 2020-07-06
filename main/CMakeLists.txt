set(CMAKE_CXX_STANDARD 17)

idf_component_register(SRCS amice.cpp
            api/api_context.h api/api_context.cpp api/api_server.h api/bus_status_handler.h api/bus_status_handler.cpp api/can_data_handler.h api/ping_handler.h api/ping_handler.cpp
                api/can_data/era_can_data.h
            application/can_events.h application/can_reader_task.h application/can_reader_task.cpp application/can_store_task.h application/can_store_task.cpp application/status_monitor_task.h application/status_monitor_task.cpp
            can/can_buffer.h can/can_buffer.cpp can/can_bus_status.h can/can_bus_status.cpp can/can_frame.h can/can_frame.cpp can/can_store.h can/can_store.cpp
            canspec/era/era_canspec.h canspec/era/era_matcher.h canspec/era/era_matcher.cpp
            core/event_group.h core/mutex.h core/mutex.cpp core/task.h core/task.cpp core/timer.h core/timer.cpp core/utils.h core/utils.cpp
            http_server/embedded_file_handler.h http_server/server.h
            io/gpo.h io/gpo.cpp
            wifi/access_point.h wifi/access_point.cpp wifi/station.h wifi/station.cpp
        INCLUDE_DIRS ".")