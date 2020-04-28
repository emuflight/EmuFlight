#ifndef SPECTROGRAPH_H
#define SPECTROGRAPH_H


#define SPEC_FFT_LENGTH 512

enum SpecCommand {
    SPEC_COMMAND_NONE,
    SPEC_COMMAND_SWAXIS,
};

void spectrographInit();
void spectrographMain();
void spectrographOSD(enum SpecCommand command);
#endif /* SPECTROGRAPH_H */

