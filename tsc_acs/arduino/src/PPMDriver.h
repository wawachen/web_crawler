/*H
Copyright 2017 Transport Systems Catapult (The Catapult)
All rights reserved
For use only for the purpose agreed with The Catapult
H*/

#pragma once

#include <inttypes.h>
#include <string.h>
#include <Arduino.h>

namespace PPM
{
// Non-template base class required to allow singleton.
class ReaderBase
{
public:
	virtual void risingEdge() = 0;
	static ReaderBase * singleton;
};

namespace ISR
{
void risingEdge() { ReaderBase::singleton->risingEdge(); }
}

int digitalPinToInterrupt(uint8_t pin)
{
	return (pin - 2);
}

// Class that times pulses in PPM sequence and provides access to only the most recent, complete
// and valid block of channels.
template <int CHANNEL_COUNT>
class Reader : public ReaderBase
{
public:
	Reader(uint8_t pin)
: pin(pin), valueTime(0), currentPulse(CHANNEL_COUNT), pulseTime(0), readCount(0),
  syncRepeats(0), blockCount(0)

{
		for (int i=0; i<CHANNEL_COUNT; i++)
			values[i] = 0;

		pinMode(pin, INPUT);
		ReaderBase::singleton = this;
		attachInterrupt(digitalPinToInterrupt(pin), ISR::risingEdge, RISING);
}
	~Reader()
	{
		detachInterrupt(digitalPinToInterrupt(pin));
	}

	void getValues(int16_t * v, uint32_t & age, uint32_t & blocks, uint32_t & count, uint32_t & repeats)
	{
		uint32_t readCountStart;

		// Record the value of readCount before the copy.
		// If the readCount has changed after the copy the start again.
		// The change implies that a new set of values arrived mid-copy.
		do
		{
			readCountStart = readCount;
			for (int i=0; i<CHANNEL_COUNT; i++)
				v[i] = this->values[i];
			age = micros() - valueTime;
			blocks = blockCount;
			count = readCount;
			repeats = syncRepeats;
			syncRepeats++;
		} while (readCountStart != readCount);
		syncRepeats--;
	}

	virtual void risingEdge()	// interrupt handler
	{
		uint32_t now = micros();
		uint32_t period = now - pulseTime;
		pulseTime = now;

		if (period > channelBlank)
		{
			// Start of new block of values
			currentPulse = 0;
			blockCount++;
		}
		else if (currentPulse < CHANNEL_COUNT)
		{
			if (period < channelZero - channelMaxError || period > channelFull + channelMaxError)
			{
				// Value out of range, scrap all readings
				currentPulse = CHANNEL_COUNT;
			}
			else
			{
				currentValues[currentPulse] = (int16_t)(period - channelZero);
				currentPulse++;

				if (currentPulse == CHANNEL_COUNT)
				{
					readCount++;
					valueTime = now;
					for (int i=0; i<CHANNEL_COUNT; i++)
						values[i] = currentValues[i];
				}
			}
		}
	}

private:
	const uint32_t channelZero = 1000;		// duration in us of zero pulse
	const uint32_t channelFull = 2000;		// duration in us of max length pulse
	const uint32_t channelMaxError = 10;	// maximum outside zero-full allowed.
	const uint32_t channelBlank = 2100;		// minimum duration in us before first channel

	uint8_t pin;	// pin on which signal is connected

	volatile int16_t values[CHANNEL_COUNT];			// most recent channel values
	volatile int16_t currentValues[CHANNEL_COUNT];	// values currently being read
	volatile uint32_t valueTime;					// time in us values received
	volatile uint8_t currentPulse;						// current channel number
	volatile uint32_t pulseTime;					// time in us of last pulse

	volatile uint32_t blockCount; // number of blocks started
	volatile uint32_t readCount;  // number of valid blocks completed
	uint32_t syncRepeats;         // number of times values get was repeated due to being interrupted by block completion.
};
}
