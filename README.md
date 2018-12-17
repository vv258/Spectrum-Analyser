Who doesn't like to have a personal assistant to do things for you and answer
your queries? The growing popularity of Alexa, Sir, Cortana, Google Assistant
and all others are testaments to this booming trend. Speech recognition is the
ability of a machine to understand spoken words and sentences. The motivation
for this project is to implement a front end for Speech Recognition engines.

Speech Recognition is a highly complex process, which is not possible to fully
complete within the allotted 5-week period. Also, the complex computation
involved in this may be too intricate to implement using a single PIC32
microcontroller. However, the analog interface and real-time nature of PIC32
microcontroller makes it suitable for real-time acquisition of the audio
signal which is the first major stage in Speech recognition. Moreover, the
serial interface can be used to connect the front end to further stages for
processing.

![](http://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2018/vv2
58_gsb85/vv258_gsb85/spectrum_analyser_files/highleveldesign.png)

#### Audio input

The audio signal is received through a microphone and DC shifted using a bias
voltage, amplified and fed to the ADC of PIC32 microcontroller.

#### Analog to Digital Conversion

The continuous time continuous amplitude analog signal is converted to
discrete time discrete amplitude digital signal using the internal ADC of
PIC32 microcontroller. The closeness of digital value to the actual analog
signal is determined by 2 factors. First, is bit depth and second is the
sampling frequency. The ADC converts the signal to 10 bit values. The sampling
rate is set to 16 Khz. he Nyquist-Shannon sampling theorem states that in
order to accurately reconstruct a signal of a specified bandwidth the sampling
frequency must be greater than twice the highest frequency of the signal being
sampled. This theoretically gives a range of 0 to 8 Khz with sampling
frequency 16 Khz. However, for speech signal processing 4 Khz is sufficient.

#### Hanning Window

The analysis of a signal can be performed by taking a finite set of samples at
a time. But, when continuous signal is broken down into sets, it results in
discontinuities at the ends. If analysis is carried out directly, these
discontinuities would appear as spurious high frequency signals. To avoid this
the amplitude of the discontinuities at ends are reduced by multiplying by a
sequence. This sequence is called window function. One such window is a
hanning window, which has a sinusoidal shape and touches zero at both ends.

It is given by:

![](http://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2018/vv2
58_gsb85/vv258_gsb85/spectrum_analyser_files/hanningformula.png)

![](http://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2018/vv2
58_gsb85/vv258_gsb85/spectrum_analyser_files/hanningwindow.png)

The Fourier transform is a mathematical algorithm that converts a time domain
signal into its frequency representation. The DFT is obtained by decomposing a
sequence of values into components of different frequencies. This operation is
useful in many fields but computing it directly from the definition is often
too slow to be practical. An FFT rapidly computes such transformations by
factorizing the DFT matrix into a product of sparse (mostly zero) factors. The
difference in speed can be enormous, especially for long data sets where N may
be in the thousands or millions. In the presence of round-off error, many FFT
algorithms are also much more accurate than evaluating the DFT definition
directly. Using Discrete FFT, 256 samples of the audio signal are converted to
256 discrete frequency points. A fixed-point number based decimation-in-time
FFT algorithm adapted from Bruce Land is used to convert the discrete digital
audio signal into discrete frequency domain.

#### Power Spectrum

The Power Spectrum gives the how much of the signal is at a particular
frequency. A PSD is computed by multiplying each frequency bin in an FFT by
its complex conjugate which results in the real only spectrum of amplitude
square.

![](http://people.ece.cornell.edu/land/courses/ece4760/FinalProjects/f2018/vv2
58_gsb85/vv258_gsb85/spectrum_analyser_files/powerspectrum.png)

The amplitude spectrum can be obtained by taking square root of this value.
However, direct computation of square root is not efficient in the PIC32
architecture. Hence, fast square root approximation technique is used.

#### MEL Filter

The human ear cannot differentiate between two closely spaced frequencies.
This effect is increase as the frequency increases. Mel Filter banks are used
to bin together the frequencies in a non linear manner. Then bins are narrower
at lower frequencies and wider at high frequencies. This helps to visualize
sound as perceived by the human ear.

The formula for converting from frequency to Mel scale is: ![](http://people.e
ce.cornell.edu/land/courses/ece4760/FinalProjects/f2018/vv258_gsb85/vv258_gsb8
5/spectrum_analyser_files/melfilter.png)

For speech signal the range of frequencies to be represented is chosen from
300 Hz to 4000 Hz.

Converting this to MEL scale we get 402 - 2146 as MEL frequency range To
obtain 20 bins, the MEL range is divide into 21 linear values. This gives the
MEL frequency bins between the following points. 402 , 485 , 568 , 651 , 734 ,
817 , 900 , 983 , 1066 , 1149 , 1232 , 1316 , 1399 , 1482 , 1565 , 1648 , 1731
, 1814 , 1897 , 1980 , 2063 , 2146

To go from Mels back to frequency: ![](http://people.ece.cornell.edu/land/cour
ses/ece4760/FinalProjects/f2018/vv258_gsb85/vv258_gsb85/spectrum_analyser_file
s/melfre.png)

This gives frequencies at: 300 , 376 , 459 , 547 , 643 , 746 , 856 , 975 ,
1103 , 1241 , 1390 , 1549 , 1721 , 1907 , 2106 , 2320 , 2551 , 2800 , 3068 ,
3356 , 3666 , 4000 The frequencies in FFT number is obtained by multiplying
with Number of samples (256) and diving by sample rate (16Khz). This gives:

19 , 24 , 29 , 35 , 41 , 47 , 54 , 62 , 70 , 79 , 89 , 99 , 110 , 122 , 135 ,
148 , 163 , 179 , 196 , 215 , 235 , 256

The first filterbank will start at the first point, reach its peak at the
second point, then return to zero at the 3rd point. The second filterbank will
start at the 2nd point, reach its max at the 3rd, then be zero at the 4th etc.
A formula for calculating these is as follows: ![](http://people.ece.cornell.e
du/land/courses/ece4760/FinalProjects/f2018/vv258_gsb85/vv258_gsb85/spectrum_a
nalyser_files/fft.png)  
where 'M' is the number of filters we want, and f() is the list of M+2 Mel-
spaced frequencies.

The project is highly software intensive. Not much hardware/ software
tradeoffs were involved in this. However, being computation intensive, code
optimization is very important. Algorithm selection was carried out by
considering the hardware limitations in the microcontroller. In the above
description, two such algorithms are used for efficient computation. 1. Fast
Fourier Transform
