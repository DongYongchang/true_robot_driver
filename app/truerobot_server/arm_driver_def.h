#pragma once

namespace arm_driver {
enum class State
{
    None,
    Init,
    Stop,
    Run,
    Error,
    Reset,
};

enum class Command
{
    None,
    Init,
    Start,
    Stop,
    Reset
};

} // namespace arm_driver