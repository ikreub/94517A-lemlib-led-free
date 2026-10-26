#pragma once

#include <cstdint>

namespace team_ui {

    enum class Alliance : std::uint8_t {
        RED,
        BLUE
    };

    void initialize();
    Alliance getAlliance();
    int getSelectedAuton();
    bool is_locked();
    void waitForUnlock();
    bool consumeManualStartRequest();

} // namespace team_ui