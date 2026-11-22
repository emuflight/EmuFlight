#include "interface/msp.c"

void testMspReboot(uint8_t mode)
{
    rebootMode = mode;
    mspRebootFn(NULL);
}

uint8_t testMspRebootFirmwareMode(void)
{
    return MSP_REBOOT_FIRMWARE;
}
