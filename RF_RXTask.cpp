#include "RF_RXTask.hpp"
#include "TCHandlingTask.hpp"
#include "PlatformParameters.hpp"
#include "etl/crc32_c.h"


SpacecraftErrorCode RF_RXTask::physicalLayerFilter(uint8_t* buf, bool use_crc, bool use_scramble) {
    // BASIC CHECKS

    if (use_scramble) {
        for (size_t i = 0; i < FIXED_LENGTH_AT_SIZE_RX; i++) {
            buf[i] = buf[i] ^ LUT[i % 255];
        }
    }
    if (use_crc) {
        uint8_t r_b0 = buf[FIXED_LENGTH_AT_SIZE_RX - 4];
        uint8_t r_b1 = buf[FIXED_LENGTH_AT_SIZE_RX - 3];
        uint8_t r_b2 = buf[FIXED_LENGTH_AT_SIZE_RX - 2];
        uint8_t r_b3 = buf[FIXED_LENGTH_AT_SIZE_RX - 1];

        uint32_t received_crc =
            (static_cast<uint32_t>(r_b0) << 24) |
            (static_cast<uint32_t>(r_b1) << 16) |
            (static_cast<uint32_t>(r_b2) << 8) |
            (static_cast<uint32_t>(r_b3));

        uint32_t computed_crc = etl::crc32_c(buf, buf + FIXED_LENGTH_AT_SIZE_RX - sizeof(uint32_t)).value();

        // Break computed CRC into bytes
        uint8_t c_b0 = static_cast<uint8_t>((computed_crc >> 24) & 0xFF);
        uint8_t c_b1 = static_cast<uint8_t>((computed_crc >> 16) & 0xFF);
        uint8_t c_b2 = static_cast<uint8_t>((computed_crc >> 8) & 0xFF);
        uint8_t c_b3 = static_cast<uint8_t>(computed_crc & 0xFF);

        // // Print the bytes (decimal)
        // LOG_DEBUG << "[RX] CRC bytes (received): "
        //           << static_cast<int>(r_b0) << " "
        //           << static_cast<int>(r_b1) << " "
        //           << static_cast<int>(r_b2) << " "
        //           << static_cast<int>(r_b3);
        //
        // LOG_DEBUG << "[RX] CRC bytes (computed): "
        //           << static_cast<int>(c_b0) << " "
        //           << static_cast<int>(c_b1) << " "
        //           << static_cast<int>(c_b2) << " "
        //           << static_cast<int>(c_b3);

        if (computed_crc != received_crc) {
            // LOG_ERROR << "[RX] CRC mismatch, computed: " << computed_crc;
            // LOG_ERROR << "[RX] CRC mismatch, received: " << received_crc;
            return TTC_ERROR_RX_WRONG_CRC;
        }
    }

    return GENERIC_ERROR_NONE;
}


SpacecraftErrorCode RF_RXTask::routePacket(const uint8_t* rx_buf, uint16_t ccsds_length, uint16_t ms_to_wait_if_queue_is_full) {
    SpacecraftErrorCode spacecraft_error_code = GENERIC_ERROR_NONE;
    PacketHandler tc_packet_handler{};
    memcpy(tc_packet_handler.buf, rx_buf, ccsds_length);
    tc_packet_handler.data_length = ccsds_length;
    auto status = xQueueSendToBack(TCQueue, &tc_packet_handler, ms_to_wait_if_queue_is_full);
    if (status == pdTRUE) {
        tcHandlingTask->rf_rx_active_ = true;
        if (tcHandlingTask->taskHandle != nullptr)
            xTaskNotifyIndexed(tcHandlingTask->taskHandle, NOTIFY_INDEX_INCOMING_TC, TC_RF_RX, eSetBits);
    } else {
        return TTC_ERROR_TC_QUEUE_FULL;
    }
    return spacecraft_error_code;
}


SpacecraftErrorCode RF_RXTask::filterCCSDSPrimaryHeader(const uint8_t* buf, CCSDSHEADER& header) {
    header.raw_id = ((buf[0] & 0x07) << 8) | buf[1];
    header.application_process_ID = (header.raw_id >> 9) & 0x3;
    header.spacecraft_ID = header.raw_id & 0x01FF;
    header.packet_data_length = (buf[4] << 8) | buf[5];

    if (header.spacecraft_ID != SpacecraftID) {
        return TTC_ERROR_RX_WRONG_SPACECRAFT_ID;
    }
    if (header.application_process_ID != TTC_APPLICATION_ID && header.application_process_ID != OBC_APPLICATION_ID) {
        return TTC_ERROR_RX_WRONG_APPLICATION_ID;
    }
    uint16_t ccsds_length = header.packet_data_length + CCSDSPrimaryHeaderSize + 1;
    if (ccsds_length > FIXED_LENGTH_AT_SIZE_RX - CRC_LENGTH) {
        return TTC_ERROR_RX_WRONG_SIZE_UP;
    }
    header.ccsds_length = ccsds_length;
    return GENERIC_ERROR_NONE;
}


[[noreturn]] void RF_RXTask::execute() {
    RFRXTaskState current_state_sub_state_ = RFRXTaskState::DISABLED;
    SpacecraftErrorCode spacecraft_error_code = GENERIC_ERROR_NONE;
    StateChangeMessage state_change_message = StateChangeFactory::FromRfrxState(current_state_sub_state_);
    CCSDSHEADER ccsdheader{0, 0, 0, 0, 0};

    while (true) {
        switch (current_state_sub_state_) {
            case RFRXTaskState::NOMINAL: {
                LOG_INFO << "[RX NOMINAL]";
                At86rf215::turnOnUHFRXAmp();
                COMMSParameters::COMMS_RX_TASK_STATE = COMMSParameters::RXTaskNominal;
                transceiver.setFrequency(COMMSParameters::COMMS_RF_RX_FREQUENCY, error_);
                if (error_ != NO_ERRORS) {
                    UnifiedModuleError unified(error_);
                    spacecraft_error_code = mapSpacecraftErrorCode(unified);
                    REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                    // reportEvent(EventReportService::RXTaskEvent, "CouldNotChangeFrequencyInit", EventSeverity::HighSeverity, true);
                    reportComponentFailure(Component::RF_TRANSCEIVER);
                }
                // ALWAYS AFTER SET FREQUENCY
                At86rf215::ensureRXModeUpdated(error_, false);
                if (error_ != NO_ERRORS) {
                    UnifiedModuleError unified(error_);
                    spacecraft_error_code = mapSpacecraftErrorCode(unified);
                    REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                    // reportEvent(EventReportService::RXTaskEvent, "CouldNotGetIntoRXInit", EventSeverity::HighSeverity, true);
                    reportComponentFailure(Component::RF_TRANSCEIVER);
                }

                while (true) {
                    xQueueReceive(rf_rx_state_change_queue, &state_change_message, pdMS_TO_TICKS(WAIT_FOR_STATE_CHANGE_FROM_NOMINAL_TIME_MS));
                    if (state_change_message.newRfrxState != current_state_sub_state_) {
                        LOG_ERROR << "[RX NOMINAL] changing state...";
                        current_state_sub_state_ = state_change_message.newRfrxState;
                        break;
                    }
                    if (xTaskNotifyWaitIndexed(NOTIFY_INDEX_RXFE_RX, pdFALSE, 0xFFFFFFFF, &received_events_, pdMS_TO_TICKS(WAIT_FOR_NOTIFICATION)) == pdTRUE) {
                        if (xSemaphoreTake(transceiver_handler.task_level_resources_mtx, portMAX_DELAY) == pdTRUE) {
                            // LOG_DEBUG << "RX: RSSI: " << COMMSParameters::COMMS_LAST_RSSI;
                            // // Read the buf
                            transceiver.readRxBuf(g_rx_buf, FIXED_LENGTH_AT_SIZE_RX, error_);
                            // Parse & filter the packet
                            if (error_ != NO_ERRORS) {
                                UnifiedModuleError unifiedErr(error_);
                                spacecraft_error_code = mapSpacecraftErrorCode(unifiedErr);
                                // report the error
                                REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                            }
                            COMMSParameters::COMMS_RF_RX_TOTAL_RECEIVED_PACKETS = COMMSParameters::COMMS_RF_RX_TOTAL_RECEIVED_PACKETS + 1;
                            rf_rxtask->incrementCounter(RfRXCounterType::ReceivedPackets);
                            spacecraft_error_code = physicalLayerFilter(g_rx_buf, COMMSParameters::COMMS_RF_CRC_ACTIVE, true);
                            // Counters handling
                            switch (spacecraft_error_code) {
                                case GENERIC_ERROR_NONE:
                                    rf_rxtask->incrementCounter(RfRXCounterType::ReceivedOkPackets);
                                    COMMSParameters::COMMS_RF_RX_ΟΚ_RECEIVED_PACKETS = COMMSParameters::COMMS_RF_RX_ΟΚ_RECEIVED_PACKETS + 1;
                                    break;
                                case TTC_ERROR_RX_WRONG_CRC:
                                    rf_rxtask->incrementCounter(RfRXCounterType::WrongCRCPackets);
                                    COMMSParameters::COMMS_RF_RX_INVALID_CRC_PACKETS = COMMSParameters::COMMS_RF_RX_INVALID_CRC_PACKETS + 1;
                                    break;
                                default:
                                    break;
                            }
                            // printCounters();
                            if (spacecraft_error_code)
                                REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                            //
                            else {
                                spacecraft_error_code = filterCCSDSPrimaryHeader(g_rx_buf, ccsdheader);
                                if (spacecraft_error_code) {
                                    REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                                }
                                else {
                                    spacecraft_error_code = routePacket(g_rx_buf, ccsdheader.ccsds_length, 100);
                                    if (spacecraft_error_code)
                                        REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                                }
                            }
                            At86rf215::ensureRXModeUpdated(error_, false);
                            if (error_ != NO_ERRORS) {
                                UnifiedModuleError unifiedErr(error_);
                                spacecraft_error_code = mapSpacecraftErrorCode(unifiedErr);
                                // Report the error
                                REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, true, false, 100);
                                reportComponentFailure(Component::RF_TRANSCEIVER);
                            }
                            xSemaphoreGive(transceiver_handler.task_level_resources_mtx);
                        }
                    }
                }
                break;
            }
            case RFRXTaskState::SAFE: {
                COMMSParameters::COMMS_RX_TASK_STATE = (COMMSParameters::RXTaskSafe);
                At86rf215::turnOffUHFRXAmp();
                while (true) {
                    LOG_ERROR << "[RX SAFE]";
                    xQueueReceive(rf_rx_state_change_queue, &state_change_message, pdMS_TO_TICKS(WAIT_FOR_STATE_CHANGE_FROM_SAFE_TIME_MS));
                    if (state_change_message.newRfrxState != current_state_sub_state_) {
                        LOG_DEBUG << "[RX SAFE]: received state change";
                        current_state_sub_state_ = state_change_message.newRfrxState;
                        break; // Exit Safe Mode
                    }
                }
                break;
            }
            case RFRXTaskState::DISABLED: {
                At86rf215::turnOffUHFRXAmp();
                COMMSParameters::COMMS_RX_TASK_STATE = (COMMSParameters::RXTaskDisabled);
                while (true) {
                    LOG_ERROR << "[RX DISABLED]";
                    xQueueReceive(rf_rx_state_change_queue, &state_change_message, pdMS_TO_TICKS(WAIT_FOR_STATE_CHANGE_FROM_DISABLED_TIME_MS));
                    if (state_change_message.newRfrxState != current_state_sub_state_) {
                        LOG_DEBUG << "[RX DISABLED]: received state change";
                        current_state_sub_state_ = state_change_message.newRfrxState;
                        break; // Exit Disabled Mode
                    }
                }
                break;
            }
            default:
                LOG_ERROR << "[RX] requested unknown state";
        }
    }
}
