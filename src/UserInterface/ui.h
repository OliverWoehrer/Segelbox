#ifndef UI_H
#define UI_H

#define STACK_SIZE 2048 // default stack size: 2048 words = 8MB

namespace SerialCLI {

enum Command {
    NONE,
    HELP,
    CAL,
    RUN,
    COMP,
    UNKNOWN
};

int init(TaskFunction_t receiveInterruptFunc);
Command parseCommand(const char* command);
Command getCurrentCommand();

}

#endif /* UI_H */