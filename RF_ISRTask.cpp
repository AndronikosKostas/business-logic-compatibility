#include "RF_ISRTask.hpp"
#include "GlobalVariables.hpp"
#include "PlatformParameters.hpp"

[[noreturn]] void RF_ISR::execute() {
    current_state_sub_state_ = RFISRTaskState::NOMINAL;
    // Initialize state
    StateChangeMessage state_change_message = StateChangeFactory::FromRfisrState(current_state_sub_state_);
    while (true) {
        switch (current_state_sub_state_) {
            case RFISRTaskState::NOMINAL: {
                LOG_DEBUG << "[ISR NOMINAL]";
                while (true) {
                    if (xTaskNotifyWaitIndexed(NOTIFY_INDEX_RF_ISR, pdFALSE, 0xFFFFFFFF, &received_events_, portMAX_DELAY) == pdTRUE) {
                        uint8_t state = transceiver.get_state(RF09, error_);
                        uint8_t invalid_retries = 6;

                        while (state == static_cast<uint8_t>(RF_INVALID) && invalid_retries > 0) {
                            state = transceiver.get_state(RF09, error_);
                            vTaskDelay(pdMS_TO_TICKS(10));
                            invalid_retries--;
                        }
                        if (invalid_retries == 0) {
                            LOG_ERROR << "[ISR] Timeout waiting for valid state, skipping IRQ handling";
                            REPORT_ERROR_WITH_CONTEXT(TTC_ERROR_RF_ISR_TIMEOUT, false, false, 1);
                            reportComponentFailure(Component::RF_TRANSCEIVER);
                        } else {
                            transceiver.handleIrq(error_);
                            if (error_ != NO_ERRORS) {
                                UnifiedModuleError unified(error_);
                                SpacecraftErrorCode spacecraft_error_code = mapSpacecraftErrorCode(unified);
                                REPORT_ERROR_WITH_CONTEXT(spacecraft_error_code, false, false, 1);
                                reportComponentFailure(Component::RF_TRANSCEIVER);
                            }
                        }
                    }
                    // Check for state change messages (non-blocking)
                    if (xQueueReceive(rf_isr_state_change_queue, &state_change_message, 0) == pdTRUE) {
                        if (state_change_message.newRfisrState != current_state_sub_state_) {
                            LOG_DEBUG << "[ISR NOMINAL]: received state change";
                            current_state_sub_state_ = state_change_message.newRfisrState;
                            break; // Exit NOMINAL
                        }
                    }
                }
                break;
            }

            case RFISRTaskState::SAFE: {
                while (true) {
                    LOG_ERROR << "[ISR SAFE]";
                    xQueueReceive(rf_isr_state_change_queue, &state_change_message, pdMS_TO_TICKS(WAIT_FOR_STATE_CHANGE_FROM_SAFE_MS));
                    if (state_change_message.newRfisrState != current_state_sub_state_) {
                        LOG_DEBUG << "[ISR SAFE]: received state change";
                        current_state_sub_state_ = state_change_message.newRfisrState;
                        break; // Exit Safe Mode
                    }
                }
                break;
            }
            case RFISRTaskState::DISABLED: {
                while (true) {
                    LOG_ERROR << "[ISR DISABLED]";
                    xQueueReceive(rf_isr_state_change_queue, &state_change_message, pdMS_TO_TICKS(WAIT_FOR_STATE_CHANGE_FROM_DISABLED_MS));
                    if (state_change_message.newRfisrState != current_state_sub_state_) {
                        LOG_DEBUG << "[ISR DISABLED]: received state change";
                        break;
                    }
                }
                break;
            }
        }
    }
}

