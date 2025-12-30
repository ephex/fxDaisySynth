#include "daisy_seed.h"
#include "daisysp.h"
#include "daisysp-lgpl.h"
#include "daisy_pod.h"
#include <map>

using namespace daisy;
using namespace daisysp;

static DaisyPod   pod;
static Oscillator osc, lfo;
static MoogLadder flt, fltLow;
static AdEnv      ad;
MidiUsbHandler    midi;
static Parameter  pitchParam, cutoffParam, resParam, lfoParam;

int   wave, mode;
float vibrato, oscFreq, lfoFreq, lfoAmp, attack, release, cutoff, res;
float oldk1, oldk2, k1, k2;
bool  selfCycle;

void ConditionalParameter(float  oldVal,
                          float  newVal,
                          float &param,
                          float  update);

void Controls();

void NextSamples(float &sig)
{
    float ad_out = ad.Process();
    vibrato      = lfo.Process();

    osc.SetFreq(oscFreq + vibrato);

    sig = osc.Process();

    // My attempt at the "ResBass" feature on the Moog Messenger.
    // At a certain low frequency threshold, start re-introducing
    // the raw signal but passed through a second filter with
    // same cutoff but zero resonance.
    // The two are combined using a formula.
    // (linear: threshold - cutoff / threshold).
    /*
    float threshold = 260.0f;
    if (cutoff < threshold) {
        sig = (
            (flt.Process(sig) * ( 1 - ( (threshold-cutoff) / threshold ) ) ) + 
            (fltLow.Process(sig) * ( (threshold-cutoff) / threshold ) )
        );
    }
    else {
        sig = flt.Process(sig);
    }
    */
    // As resonance increases, introduce more of the pre-filter signal,
    // but pass that through another filter that has no resonance.
    float threshold = 0.5f;
    if (res > threshold) {
        sig = (
            (flt.Process(sig) * ( 1 - ( (threshold-res) / threshold ) ) ) + 
            (fltLow.Process(sig) * ( (threshold-res) / threshold ) )
        );
    }
    else {
        sig = flt.Process(sig);
    }

    
    sig *= ad_out;
}

static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    Controls();

    for(size_t i = 0; i < size; i += 2)
    {
        float sig;
        NextSamples(sig);

        // left out
        out[i] = sig;

        // right out
        out[i + 1] = sig;
    }
}

// Typical Switch case for Message Type.
void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent note_msg = m.AsNoteOn();
            char        buff[512];
            sprintf(buff,
                    "Note Received:\t%d\t%d\t%d\r\n",
                    m.channel,
                    m.data[0],
                    m.data[1]);
            pod.seed.usb_handle.TransmitInternal((uint8_t *)buff, strlen(buff));
            // This is to avoid Max/MSP Note outs for now..
            if(m.data[1] != 0)
            {
                note_msg = m.AsNoteOn();
                osc.SetFreq(mtof(note_msg.note));
                oscFreq = mtof(note_msg.note);
                osc.SetAmp((note_msg.velocity / 127.0f));
                ad.Trigger();
            }
        }
        break;
        case ControlChange:
        {
            ControlChangeEvent p = m.AsControlChange();
            switch(p.control_number)
            {
                case 1:
                    // CC 1 for cutoff.
                    flt.SetFreq(mtof((float)p.value));
                    fltLow.SetFreq(mtof((float)p.value)+100.0f);
                    break;
                case 2:
                    // CC 2 for res.
                    flt.SetRes(((float)p.value / 127.0f));
                    break;
                default: break;
            }
            break;
        }
        default: break;
    }
}


int main(void)
{
    // Set global variables
    float sample_rate;
    mode    = 0;
    vibrato = 0.0f;
    oscFreq = 1000.0f;
    oldk1 = oldk2 = 0;
    k1 = k2   = 0;
    attack    = .01f;
    release   = .8f;
    cutoff    = 10000;
    lfoAmp    = 1.0f;
    lfoFreq   = 0.1f;
    selfCycle = false;

    //Init everything
    pod.Init();
    pod.SetAudioBlockSize(4);
    sample_rate = pod.AudioSampleRate();
    osc.Init(sample_rate);
    flt.Init(sample_rate);
    fltLow.Init(sample_rate);
    ad.Init(sample_rate);
    lfo.Init(sample_rate);

    //Set filter parameters
    flt.SetFreq(10000);
    flt.SetRes(0.8);
    fltLow.SetFreq(1100);
    fltLow.SetRes(0.0);

    // Set parameters for oscillator
    osc.SetWaveform(osc.WAVE_SAW);
    wave = osc.WAVE_SAW;
    osc.SetFreq(440);
    osc.SetAmp(1);

    // Set parameters for lfo
    lfo.SetWaveform(osc.WAVE_SIN);
    lfo.SetFreq(0.1);
    lfo.SetAmp(1);

    //Set envelope parameters
    ad.SetTime(ADENV_SEG_ATTACK, 0.01);
    ad.SetTime(ADENV_SEG_DECAY, .8);
    ad.SetMax(1);
    ad.SetMin(0);
    ad.SetCurve(0.5);

    //set parameter parameters
    cutoffParam.Init(pod.knob1, 100, 20000, cutoffParam.LOGARITHMIC);
    resParam.Init(pod.knob2, 0.0, 1.0, resParam.LINEAR);
    lfoParam.Init(pod.knob1, 0.25, 1000, lfoParam.LOGARITHMIC);

    /** Initialize USB Midi 
     *  by default this is set to use the built in (USB FS) peripheral.
     * 
     *  by setting midi_cfg.transport_config.periph = MidiUsbTransport::Config::EXTERNAL
     *  the USB HS pins can be used (as FS) for MIDI 
     */
    MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
    midi.Init(midi_cfg);

    // start callback
    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    while(1) {
        /** Listen to MIDI for new changes */
        midi.Listen();

        /** When there are messages waiting in the queue... */
        while(midi.HasEvents())
        {
            HandleMidiMessage(midi.PopEvent());
        }
    }
}

//Updates values if knob had changed
void ConditionalParameter(float  oldVal,
                          float  newVal,
                          float &param,
                          float  update)
{
    if(abs(oldVal - newVal) > 0.00005)
    {
        param = update;
    }
}


//Controls Helpers
void UpdateEncoder()
{
    wave += pod.encoder.RisingEdge();
    wave %= osc.WAVE_POLYBLEP_TRI;

    //skip ramp since it sounds like saw
    if(wave == 3)
    {
        wave = 4;
    }

    osc.SetWaveform(wave);

    mode += pod.encoder.Increment();
    mode = (mode % 3 + 3) % 3;
}

void UpdateKnobs()
{
    k1 = pod.knob1.Process();
    k2 = pod.knob2.Process();

    switch(mode)
    {
        case 0:
            ConditionalParameter(oldk1, k1, cutoff, cutoffParam.Process());
            ConditionalParameter(oldk2, k2, res, resParam.Process());
            flt.SetFreq(cutoff);
            fltLow.SetFreq(cutoff+100.0f);
            flt.SetRes(res);
            break;
        case 1:
            ConditionalParameter(oldk1, k1, attack, pod.knob1.Process());
            ConditionalParameter(oldk2, k2, release, pod.knob2.Process());
            ad.SetTime(ADENV_SEG_ATTACK, attack);
            ad.SetTime(ADENV_SEG_DECAY, release);
            break;
        case 2:
            ConditionalParameter(oldk1, k1, lfoFreq, lfoParam.Process());
            ConditionalParameter(oldk2, k2, lfoAmp, pod.knob2.Process());
            lfo.SetFreq(lfoFreq);
            lfo.SetAmp(lfoAmp * 100);
        default: break;
    }
}

void UpdateLeds()
{
    pod.led1.Set(mode == 2, mode == 1, mode == 0);
    pod.led2.Set(0, selfCycle, selfCycle);

    oldk1 = k1;
    oldk2 = k2;

    pod.UpdateLeds();
}

void UpdateButtons()
{
    if(pod.button1.RisingEdge() || (selfCycle && !ad.IsRunning()))
    {
        ad.Trigger();
    }

    if(pod.button2.RisingEdge())
    {
        selfCycle = !selfCycle;
    }
}

void Controls()
{
    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    UpdateEncoder();

    UpdateKnobs();

    UpdateLeds();

    UpdateButtons();
}