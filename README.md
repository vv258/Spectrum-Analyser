
Spectrum Analyser
-----------------

The Audio Spectrum Analyzer based on PIC32 microcontroller has the following features:

*   Real time speech signal acquisition
*   Spectrum and spectrograph visualization of Mel Frequency bands
*   Serial interface for forwarding Mel Frequency Band Power Spectrum coefficients
*   Recording and playback of speech signal using serial RAM
*   Spectrum Visualization on PC using MATLAB serial interface

![Block Diagram](https://github.com/vv258/Spectrum-Analyser/blob/master/images/1.png)


The system uses a microphone to pick up speech signal. The analog audio signal is sampled using ADC of PIC32 microcontroller. The signals are then converted into short windows and Fourier Transform is applied to convert from time domain to frequency domain. The FFT is then converted to Power Spectrum, followed by application of MEL filter bank. The MEL Power coefficients are plotted on the TFT screen as Spectrum or Spectrograph depending on display mode. They are also forwarded to external PC using UART interface where a MATLAB program also plots the Spectrum. In addition to the Analyze mode, Record and Playback modes are available, which records a sound signal of 5 seconds duration and does playback respectively.

![Spectrum](https://github.com/vv258/Spectrum-Analyser/blob/master/images/3.png)
![Spectrograph](https://github.com/vv258/Spectrum-Analyser/blob/master/images/4.png)

More Details at: [Hackster.io](https://www.hackster.io/128264/spectrum-analyzer-c32962 "Spectrum Analyser")
