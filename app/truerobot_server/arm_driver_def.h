#pragma once

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
