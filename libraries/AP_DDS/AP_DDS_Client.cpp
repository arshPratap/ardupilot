#include <AP_HAL/AP_HAL_Boards.h>

#if AP_DDS_ENABLED

#include <AP_GPS/AP_GPS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

#include "AP_DDS_Client.h"

static constexpr uint16_t DELAY_TIME_TOPIC_MS = 10;
static constexpr uint16_t DELAY_BATTERY_STATE_TOPIC_MS = 1000;
static char WGS_84_FRAME_ID[] = "WGS-84";
// https://www.ros.org/reps/rep-0105.html#base-link
static char BASE_LINK_FRAME_ID[] = "base_link";

const AP_Param::GroupInfo AP_DDS_Client::var_info[] {

    // @Param: _ENABLED
    // @DisplayName: DDS enable
    // @Description: Enable DDS subsystem
    // @Values: 0:Disabled,1:Enabled
    // @RebootRequired: True
    // @User: Advanced
    AP_GROUPINFO_FLAGS("_ENABLED", 1, AP_DDS_Client, enabled, 0, AP_PARAM_FLAG_ENABLE),

#if AP_DDS_UDP_ENABLED
    // @Param: _UDP_PORT
    // @DisplayName: DDS UDP port
    // @Description: UDP port number for DDS
    // @Range: 1 65535
    // @RebootRequired: True
    // @User: Standard
    AP_GROUPINFO("_PORT", 2, AP_DDS_Client, udp_port, 2019),
#endif
    
    AP_GROUPEND
};

#include "AP_DDS_Topic_Table.h"

void AP_DDS_Client::update_topic(builtin_interfaces_msg_Time& msg)
{
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    msg.sec = utc_usec / 1000000ULL;
    msg.nanosec = (utc_usec % 1000000ULL) * 1000UL;

}

bool AP_DDS_Client::update_topic(sensor_msgs_msg_NavSatFix& msg, const uint8_t instance)
{
    // Add a lambda that takes in navsatfix msg and populates the cov
    // Make it constexpr if possible
    // https://www.fluentcpp.com/2021/12/13/the-evolutions-of-lambdas-in-c14-c17-and-c20/
    // constexpr auto times2 = [] (sensor_msgs_msg_NavSatFix* msg) { return n * 2; };

    // assert(instance >= GPS_MAX_RECEIVERS);
    if (instance >= GPS_MAX_RECEIVERS) {
        return false;
    }

    auto &gps = AP::gps();
    WITH_SEMAPHORE(gps.get_semaphore());

    if (!gps.is_healthy(instance)) {
        msg.status.status = -1; // STATUS_NO_FIX
        msg.status.service = 0; // No services supported
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        return false;
    }

    // No update is needed
    const auto last_fix_time_ms = gps.last_fix_time_ms(instance);
    if (last_nav_sat_fix_time_ms == last_fix_time_ms) {
        return false;
    } else {
        last_nav_sat_fix_time_ms = last_fix_time_ms;
    }


    update_topic(msg.header.stamp);
    strcpy(msg.header.frame_id, WGS_84_FRAME_ID);
    msg.status.service = 0; // SERVICE_GPS
    msg.status.status = -1; // STATUS_NO_FIX


    //! @todo What about glonass, compass, galileo?
    //! This will be properly designed and implemented to spec in #23277
    msg.status.service = 1; // SERVICE_GPS

    const auto status = gps.status(instance);
    switch (status) {
    case AP_GPS::NO_GPS:
    case AP_GPS::NO_FIX:
        msg.status.status = -1; // STATUS_NO_FIX
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        return true;
    case AP_GPS::GPS_OK_FIX_2D:
    case AP_GPS::GPS_OK_FIX_3D:
        msg.status.status = 0; // STATUS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_DGPS:
        msg.status.status = 1; // STATUS_SBAS_FIX
        break;
    case AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT:
    case AP_GPS::GPS_OK_FIX_3D_RTK_FIXED:
        msg.status.status = 2; // STATUS_SBAS_FIX
        break;
    default:
        //! @todo Can we not just use an enum class and not worry about this condition?
        break;
    }
    const auto loc = gps.location(instance);
    msg.latitude = loc.lat * 1E-7;
    msg.longitude = loc.lng * 1E-7;

    int32_t alt_cm;
    if (!loc.get_alt_cm(Location::AltFrame::ABSOLUTE, alt_cm)) {
        // With absolute frame, this condition is unlikely
        msg.status.status = -1; // STATUS_NO_FIX
        msg.position_covariance_type = 0; // COVARIANCE_TYPE_UNKNOWN
        return true;
    }
    msg.altitude = alt_cm / 100.0;

    // ROS allows double precision, ArduPilot exposes float precision today
    Matrix3f cov;
    msg.position_covariance_type = (uint8_t)gps.position_covariance(instance, cov);
    msg.position_covariance[0] = cov[0][0];
    msg.position_covariance[1] = cov[0][1];
    msg.position_covariance[2] = cov[0][2];
    msg.position_covariance[3] = cov[1][0];
    msg.position_covariance[4] = cov[1][1];
    msg.position_covariance[5] = cov[1][2];
    msg.position_covariance[6] = cov[2][0];
    msg.position_covariance[7] = cov[2][1];
    msg.position_covariance[8] = cov[2][2];

    return true;
}

void AP_DDS_Client::populate_static_transforms(tf2_msgs_msg_TFMessage& msg)
{
    msg.transforms_size = 0;

    auto &gps = AP::gps();
    for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
        const auto gps_type = gps.get_type(i);
        if (gps_type == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
            continue;
        }
        update_topic(msg.transforms[i].header.stamp);
        char gps_frame_id[16];
        //! @todo should GPS frame ID's be 0 or 1 indexed in ROS?
        hal.util->snprintf(gps_frame_id, sizeof(gps_frame_id), "GPS_%u", i);
        strcpy(msg.transforms[i].header.frame_id, BASE_LINK_FRAME_ID);
        strcpy(msg.transforms[i].child_frame_id, gps_frame_id);
        // The body-frame offsets
        // X - Forward
        // Y - Right
        // Z - Down
        // https://ardupilot.org/copter/docs/common-sensor-offset-compensation.html#sensor-position-offset-compensation

        const auto offset = gps.get_antenna_offset(i);

        // In ROS REP 103, it follows this convention
        // X - Forward
        // Y - Left
        // Z - Up
        // https://www.ros.org/reps/rep-0103.html#axis-orientation

        msg.transforms[i].transform.translation.x = offset[0];
        msg.transforms[i].transform.translation.y = -1 * offset[1];
        msg.transforms[i].transform.translation.z = -1 * offset[2];

        msg.transforms_size++;
    }

}

void AP_DDS_Client::update_topic(sensor_msgs_msg_BatteryState& msg, const uint8_t instance)
{
    if (instance >= AP_BATT_MONITOR_MAX_INSTANCES) {
        return;
    }

    update_topic(msg.header.stamp);
    auto &battery = AP::battery();

    if (!battery.healthy(instance))
    {
        msg.power_supply_status = 3; //POWER_SUPPLY_HEALTH_DEAD
        msg.present = false;
        return;
    }
    msg.present = true;

    msg.voltage = battery.voltage(instance);

    float temperature;
    msg.temperature = (battery.get_temperature(temperature, instance)) ? temperature : NAN;

    float current;
    msg.current = (battery.current_amps(current, instance)) ? -1 * current : NAN;

    const float design_capacity = (float)battery.pack_capacity_mah(instance)/1000.0;
    msg.design_capacity = design_capacity;

    uint8_t percentage;
    if (battery.capacity_remaining_pct(percentage, instance))
    {
        msg.percentage = percentage/100.0;
        msg.charge = (percentage * design_capacity)/100.0;
    }
    else
    {
        msg.percentage = NAN;
        msg.charge = NAN;
    }

    msg.capacity = NAN;

    if (battery.current_amps(current, instance))
    {
        if (percentage == 100) {
            msg.power_supply_status = 4;   //POWER_SUPPLY_STATUS_FULL
        }
        else if (current < 0.0) {
            msg.power_supply_status = 1;   //POWER_SUPPLY_STATUS_CHARGING
        }
        else if (current > 0.0) {
            msg.power_supply_status = 2;   //POWER_SUPPLY_STATUS_DISCHARGING
        }
        else {
            msg.power_supply_status = 3;   //POWER_SUPPLY_STATUS_NOT_CHARGING
        }
    }
    else
    {
        msg.power_supply_status = 0; //POWER_SUPPLY_STATUS_UNKNOWN
    }

    msg.power_supply_health = (battery.overpower_detected(instance)) ? 4 : 1; //POWER_SUPPLY_HEALTH_OVERVOLTAGE or POWER_SUPPLY_HEALTH_GOOD

    msg.power_supply_technology = 0; //POWER_SUPPLY_TECHNOLOGY_UNKNOWN

    if (battery.has_cell_voltages(instance))
    {
        const uint16_t* cellVoltages = battery.get_cell_voltages(instance).cells;
        std::copy(cellVoltages, cellVoltages + AP_BATT_MONITOR_CELLS_MAX, msg.cell_voltage);
    }
}

/*
  start the DDS thread
 */
bool AP_DDS_Client::start(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    AP_Param::load_object_from_eeprom(this, var_info);

    if (enabled == 0) {
        return true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_DDS_Client::main_loop, void),
                                      "DDS",
                                      8192, AP_HAL::Scheduler::PRIORITY_IO, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: thread create failed");
        return false;
    }
    return true;
}

/*
  main loop for DDS thread
 */
void AP_DDS_Client::main_loop(void)
{
    if (!init() || !create()) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"DDS Client: Creation Requests failed");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Initialization passed");

    populate_static_transforms(static_transforms_topic);
    write_static_transforms();

    while (true) {
        hal.scheduler->delay(1);
        update();
    }
}

bool AP_DDS_Client::ddsSerialInit()
{
    constexpr uint8_t fd = 0;
    constexpr uint8_t relativeSerialAgentAddr = 0;
    constexpr uint8_t relativeSerialClientAddr = 1;
    if (!uxr_init_serial_transport(&serial_transport, fd, relativeSerialAgentAddr, relativeSerialClientAddr)) {
        return false;
    }
    constexpr uint32_t uniqueClientKey = 0xAAAABBBB;
    //TODO does this need to be inside the loop to handle reconnect?
    uxr_init_session(&uxr_session, &serial_transport.comm, uniqueClientKey);
    return true;
}

#if AP_DDS_UDP_ENABLED
bool AP_DDS_Client::ddsUdpInit()
{
    char port_string[10];
    hal.util->snprintf(port_string, sizeof(port_string), "%u", udp_port.get());

    if (!uxr_init_udp_transport(&udp_transport, UXR_IPv4, ip, port_string)) {
        return false;
    }
    constexpr uint32_t uniqueClientKey = 0xAAAABBBB;
    uxr_init_session(&uxr_session, &udp_transport.comm, uniqueClientKey);
    return true;
}
#endif // AP_DDS_UDP_ENABLED

bool AP_DDS_Client::init()
{
    // serial init will fail if the SERIALn_PROTOCOL is not setup
    bool initTransportStatus = ddsSerialInit();
    is_serial = initTransportStatus;

#if AP_DDS_UDP_ENABLED
    // fallback to UDP if available
    if (!initTransportStatus) {
        initTransportStatus = ddsUdpInit();
    }
#endif

    if (!initTransportStatus) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Transport Initialization failed");
        return false;
    }
    while (!uxr_create_session(&uxr_session)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Initialization waiting...");
        hal.scheduler->delay(1000);
    }

    reliable_in = uxr_create_input_reliable_stream(&uxr_session, input_reliable_stream, BUFFER_SIZE_SERIAL, STREAM_HISTORY);
    reliable_out = uxr_create_output_reliable_stream(&uxr_session, output_reliable_stream, BUFFER_SIZE_SERIAL, STREAM_HISTORY);

    if (is_serial) {
        reliable_in = uxr_create_input_reliable_stream(&uxr_session, input_reliable_stream, BUFFER_SIZE_SERIAL,STREAM_HISTORY);
        reliable_out = uxr_create_output_reliable_stream(&uxr_session, output_reliable_stream, BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    }
#if AP_DDS_UDP_ENABLED
    else {
        reliable_in = uxr_create_input_reliable_stream(&uxr_session, input_reliable_stream, BUFFER_SIZE_UDP, STREAM_HISTORY);
        reliable_out = uxr_create_output_reliable_stream(&uxr_session, output_reliable_stream, BUFFER_SIZE_UDP, STREAM_HISTORY);
    }
#endif
    reliable_in = uxr_create_input_reliable_stream(&uxr_session,input_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    reliable_out = uxr_create_output_reliable_stream(&uxr_session,output_reliable_stream,BUFFER_SIZE_SERIAL,STREAM_HISTORY);
    besteffort_in = uxr_create_input_best_effort_stream(&uxr_session);

    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"DDS Client: Init Complete");

    return true;
}

bool AP_DDS_Client::create()
{
    WITH_SEMAPHORE(csem);

    // Participant
    const uxrObjectId participant_id = {
        .id = 0x01,
        .type = UXR_PARTICIPANT_ID
    };
    const char* participant_ref = "participant_profile";
    const auto participant_req_id = uxr_buffer_create_participant_ref(&uxr_session, reliable_out, participant_id,0,participant_ref,UXR_REPLACE);

    //Participant requests
    constexpr uint8_t nRequestsParticipant = 1;
    const uint16_t requestsParticipant[nRequestsParticipant] = {participant_req_id};

    constexpr int maxTimeMsPerRequestMs = 250;
    constexpr int requestTimeoutParticipantMs = nRequestsParticipant * maxTimeMsPerRequestMs;
    uint8_t statusParticipant[nRequestsParticipant];
    if (!uxr_run_session_until_all_status(&uxr_session, requestTimeoutParticipantMs, requestsParticipant, statusParticipant, nRequestsParticipant)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Participant session request failure");
        // TODO add a failure log message sharing the status results
        return false;
    }

    for (size_t i = 0 ; i < ARRAY_SIZE(topics); i++) {
        // Topic
        const uxrObjectId topic_id = {
            .id = topics[i].topic_id,
            .type = UXR_TOPIC_ID
        };
        const char* topic_ref = topics[i].topic_profile_label;

        // Status requests
        constexpr uint8_t nRequests = 3;
        uint16_t requests[nRequests];
        if (strlen(topics[i].dw_profile_label)) {
            requests[0] = uxr_buffer_create_topic_ref(&uxr_session,reliable_out,topic_id,participant_id,topic_ref,UXR_REPLACE);

            // Publisher
            const uxrObjectId pub_id = {
                .id = topics[i].pub_id,
                .type = UXR_PUBLISHER_ID
            };
            const char* pub_xml = "";
            requests[1] = uxr_buffer_create_publisher_xml(&uxr_session,reliable_out,pub_id,participant_id,pub_xml,UXR_REPLACE);

            // Data Writer
            const char* data_writer_ref = topics[i].dw_profile_label;
            requests[2] =  uxr_buffer_create_datawriter_ref(&uxr_session,reliable_out,topics[i].dw_id,pub_id,data_writer_ref,UXR_REPLACE);
        } else if (strlen(topics[i].dr_profile_label)) {
            requests[0] = uxr_buffer_create_topic_ref(&uxr_session,reliable_in,topic_id,participant_id,topic_ref,UXR_REPLACE);

            // Subscriber
            const uxrObjectId sub_id = {
                .id = topics[i].pub_id,
                .type = UXR_SUBSCRIBER_ID
            };
            const char* sub_xml = "";
            requests[1] = uxr_buffer_create_subscriber_xml(&uxr_session,reliable_out,sub_id,participant_id,sub_xml,UXR_REPLACE);

            // Data Reader
            const char* data_reader_ref = topics[i].dr_profile_label;
            requests[2] =  uxr_buffer_create_datareader_ref(&uxr_session,reliable_out,topics[i].dw_id,sub_id,data_reader_ref,UXR_REPLACE);

        }

        constexpr int requestTimeoutMs = nRequests * maxTimeMsPerRequestMs;
        uint8_t status[nRequests];
        if (!uxr_run_session_until_all_status(&uxr_session, requestTimeoutMs, requests, status, nRequests)) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Topic/Pub/Writer session request failure for index 'TODO'");
            for (int s = 0 ; s < nRequests; s++) {
                GCS_SEND_TEXT(MAV_SEVERITY_ERROR,"XRCE Client: Status '%d' result '%u'", s, status[s]);
            }
            // TODO add a failure log message sharing the status results
            return false;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Topic/Pub/Writer session passed");
        }
    }

    uxrDeliveryControl delivery_control = {0}; // TODO tune this
    delivery_control.max_samples = UXR_MAX_SAMPLES_UNLIMITED;
    const auto datareader_id = topics[3].dw_id;
    uxr_buffer_request_data(&uxr_session, reliable_out, datareader_id, besteffort_in, &delivery_control);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"XRCE Client: Buffer request data complete");

    return true;
}

void AP_DDS_Client::write_time_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub;
        const uint32_t topic_size = builtin_interfaces_msg_Time_size_of_topic(&time_topic, 0);
        uxr_prepare_output_stream(&uxr_session,reliable_out,topics[0].dw_id,&ub,topic_size);
        const bool success = builtin_interfaces_msg_Time_serialize_topic(&ub, &time_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: XRCE_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_nav_sat_fix_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub;
        const uint32_t topic_size = sensor_msgs_msg_NavSatFix_size_of_topic(&nav_sat_fix_topic, 0);
        uxr_prepare_output_stream(&uxr_session,reliable_out,topics[1].dw_id,&ub,topic_size);
        const bool success = sensor_msgs_msg_NavSatFix_serialize_topic(&ub, &nav_sat_fix_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_static_transforms()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub;
        const uint32_t topic_size = tf2_msgs_msg_TFMessage_size_of_topic(&static_transforms_topic, 0);
        uxr_prepare_output_stream(&uxr_session,reliable_out,topics[2].dw_id,&ub,topic_size);
        const bool success = tf2_msgs_msg_TFMessage_serialize_topic(&ub, &static_transforms_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::write_battery_state_topic()
{
    WITH_SEMAPHORE(csem);
    if (connected) {
        ucdrBuffer ub;
        const uint32_t topic_size = sensor_msgs_msg_BatteryState_size_of_topic(&battery_state_topic, 0);
        uxr_prepare_output_stream(&uxr_session,reliable_out,topics[3].dw_id,&ub,topic_size);
        const bool success = sensor_msgs_msg_BatteryState_serialize_topic(&ub, &battery_state_topic);
        if (!success) {
            // TODO sometimes serialization fails on bootup. Determine why.
            // AP_HAL::panic("FATAL: DDS_Client failed to serialize\n");
        }
    }
}

void AP_DDS_Client::update()
{
    WITH_SEMAPHORE(csem);
    const auto cur_time_ms = AP_HAL::millis64();

    if (cur_time_ms - last_time_time_ms > DELAY_TIME_TOPIC_MS) {
        update_topic(time_topic);
        last_time_time_ms = cur_time_ms;
        write_time_topic();
    }

    constexpr uint8_t gps_instance = 0;
    if (update_topic(nav_sat_fix_topic, gps_instance)) {
        write_nav_sat_fix_topic();
    }

    if (cur_time_ms - last_battery_state_time_ms > DELAY_BATTERY_STATE_TOPIC_MS) {
        constexpr uint8_t battery_instance = 0;
        update_topic(battery_state_topic, battery_instance);
        last_battery_state_time_ms = cur_time_ms;
        write_battery_state_topic();
    }

    connected = uxr_run_session_time(&uxr_session, 1);
}

void AP_DDS_Client::on_topic(
        uxrSession* session,
        uxrObjectId object_id,
        uint16_t request_id,
        uxrStreamId stream_id,
        struct ucdrBuffer* ub,
        uint16_t length,
        void* args)
{
    (void) session; (void) object_id; (void) request_id; (void) stream_id; (void) length;

    // HelloWorld topic;
    // HelloWorld_deserialize_topic(ub, &topic);
    // printf("Recieved data on DDS\n");
    GCS_SEND_TEXT(MAV_SEVERITY_INFO,"Recieved Data");
    // printf("Received topic: %s, id: %i\n", topic.message, topic.index);

    // See AP_HAL::RCInput

    uint32_t* count_ptr = (uint32_t*) args;
    (*count_ptr)++;
}

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
extern "C" {
    int clock_gettime(clockid_t clockid, struct timespec *ts);
}

int clock_gettime(clockid_t clockid, struct timespec *ts)
{
    //! @todo the value of clockid is ignored here.
    //! A fallback mechanism is employed against the caller's choice of clock.
    uint64_t utc_usec;
    if (!AP::rtc().get_utc_usec(utc_usec)) {
        utc_usec = AP_HAL::micros64();
    }
    ts->tv_sec = utc_usec / 1000000ULL;
    ts->tv_nsec = (utc_usec % 1000000ULL) * 1000UL;
    return 0;
}
#endif // CONFIG_HAL_BOARD

#endif // AP_DDS_ENABLED


