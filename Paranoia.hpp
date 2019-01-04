/* Paranoia is a bit crusher, distortion, resampler and mangler.
   Copyright 2019 Daniel Arena <dan@remaincalm.org>
License: LGPL3
*/

#include "Patch.h"
#pragma once

/* start util.hpp */

typedef int samples_t; // integral sample length or position
typedef float samples_frac_t; // fractional sample length or position
typedef float signal_t; // signal value

// DB to gain coefficient

constexpr float DB_CO(float g) {
	return (g > -90.0f) ? powf(10.0f, g * 0.05f) : 0.0f;
}

// DC filter. Call process once per sample.

class DcFilter {
	public:

		signal_t process(const signal_t in) {
			out = 0.99 * out + in - prv_in;
			prv_in = in;
			return out;
		}

	private:
		signal_t out = 0;
		signal_t prv_in = 0;
};

/* SmoothParam models parameter smoothing (LERP) over a fixed # samples
 * following parameter value updates.
 */
template <class T, int U = 2400 > class SmoothParam {
	public:

		SmoothParam(T init) : value(init), start(init), end(init) {
		}

		SmoothParam<T, U>& operator=(T f) {
			start = value;
			end = f;
			t = 0;
			return *this;
		}

		SmoothParam<T, U>& operator+=(T f) {
			this->operator=(end + f);
			return *this;
		}

		SmoothParam<T, U>& operator-=(T f) {
			this->operator=(end - f);
			return *this;
		}

		operator T() const {
			return value;
		}

		void complete() {
			t = len;
			value = end;
		}

		T target() const {
			return end;
		}

		void tick() {
			if (t < len) {
				t += 1;
				const float frac = ((float) t / (float) len);
				value = (float) end * frac + (float) start * (1.0f - frac);
			} else {
				value = end;
			}
		}

	private:
		T value = 0;
		T start = 0;
		T end = 0;
		int t = 0;
		const int len = U;
};

/* end util.hpp */


/* start paranoia.hpp */


const samples_t RESAMPLE_MAX = 48000;

// waveshapes
const float PRE_SHAPER = 0.857;
const float POST_SHAPER = 0.9;
const float CLAMP = 0.9;

const int NUM_MANGLERS = 17;
const int MANGLER_BITDEPTH = 8;


// Bitcrusher.

class Mangler {
	public:

		enum Mangle {
			X = -1, // invert
			O = 0, // off
			I = 1 // on
		};

		Mangler() {
			initMangler();
		};

		// works on samples in [0, 2^bitdepth) range

		int mangleForBitDepth(const int pattern_idx, int bitdepth, const int in) const {
			int clear_mask = clear_masks_[pattern_idx];
			int xor_mask = xor_masks_[pattern_idx];

			if (bitdepth < 8) {
				clear_mask >>= 8 - bitdepth;
			}
			if (bitdepth > 8) {
				clear_mask <<= bitdepth - 8;
			}

			int curr = in & ~clear_mask;
			curr = curr ^ xor_mask;
			return curr;
		}

		float relgain(const int pattern_idx) const {
			return relgain_[pattern_idx];
		}

	private:

		void initMangler() {
			Mangle manglers[NUM_MANGLERS][MANGLER_BITDEPTH] = {
				{ I, I, I, I, I, I, I, I},
				{ I, I, I, I, I, I, I, O},
				{ I, I, I, I, I, I, I, X}, //
				{ I, I, I, I, I, I, O, I},
				{ I, I, I, I, I, I, X, I},
				{ I, I, I, I, I, O, O, I},
				{ I, I, I, I, I, I, X, X},
				{ I, I, I, X, I, I, I, I},
				{ I, I, O, I, I, I, I, I},
				{ O, X, I, O, X, O, X, I},
				{ X, X, I, I, X, I, X, I},
				{ O, O, O, O, I, O, O, O},
				{ O, O, O, O, O, X, O, I},
				{ O, O, I, I, O, O, X, I},
				{ O, O, O, I, I, X, O, X},
				{ O, O, O, O, I, I, X, X},
				{ O, O, O, O, I, I, I, I},
			};


			for (int idx = 0; idx < NUM_MANGLERS; ++idx) {
				const auto pattern = manglers[idx];
				int clear_mask = 0;
				int xor_mask = 0;
				for (int i = 0; i < 8; ++i) {
					if (pattern[8 - i] == O) {
						clear_mask = clear_mask | (int) pow(2, i);
					}
					if (pattern[8 - i] == X) {
						xor_mask = xor_mask | (int) pow(2, i);
					}
				}
				clear_masks_[idx] = clear_mask;
				xor_masks_[idx] = xor_mask;
			}
		}

		int xor_masks_[NUM_MANGLERS];
		int clear_masks_[NUM_MANGLERS];

		// gain compensation for patterns in manglers
		float relgain_[NUM_MANGLERS] = {
			1.0, // 0
			1.0,
			1.0, // 2
			0.8,
			1.0, // 4
			0.8,
			1.0, // 6
			0.1,
			0.1, // 8
			0.3,
			0.1, // 10
			1.3,
			2.0, // 12
			0.2,
			0.8, // 14
			0.5,
			0.5 // 16
		};
};

class Paranoia : public Patch {
	public:

		enum FilterMode {
			MODE_OFF = 0,
			MODE_LPF = 1,
			MODE_BANDPASS = 2,
			MODE_HPF = 3,
		};

		struct Channel {
			// filter state
			float v0 = 0;
			float v1 = 0;
			float hv0 = 0;
			float hv1 = 0;

			// resampler state
			samples_frac_t next_sample = 0;
			signal_t prev_in = 0;
			long sample_csr = 0;

			// DC filter
			DcFilter dc_filter;

		};

		struct Filter {
			SmoothParam<float> c = 0.3;
			SmoothParam<float> one_minus_rc = 0.98;

			void tick() {
				c.tick();
				one_minus_rc.tick();
			}
		};

		/**
		  Plugin class constructor.
		  You must set all parameter values to their defaults, matching the value in initParameter().
		  */
		Paranoia() : Patch() {
			srate = getSampleRate();
			/*
Level: post-saturation gain\n"
"Crush: left 300Hz-30kHz 6-bit, right 300Hz-30kHz 10-bit, far-right 48kHz 10-bit\n"
"Mangle: Sweep through bit flip/mute patterns (interactive w/ crush)\n"
"Filter: 0-80 bandpass, 80-99 highpass, 100 raw";
*/        
			registerParameter(PARAMETER_A, "Level");
			registerParameter(PARAMETER_B, "Crush");
			registerParameter(PARAMETER_C, "Mangle");
			registerParameter(PARAMETER_D, "Filter");        
		};

		void processAudio(AudioBuffer &buffer) override;

	private:
		void fixCrushParams();
		void fixFilterParams();	
		void updateParams();

		signal_t pregain(const Channel& ch, const signal_t in) const;
		signal_t resample(Channel& ch, const signal_t in) const;
		signal_t bitcrush(const signal_t in) const;
		signal_t preSaturate(const signal_t in) const;
		signal_t postSaturate(const signal_t in) const;
		signal_t filterDC(Channel& ch, const signal_t in) const;
		signal_t filterLPF(Channel& ch, const signal_t in) const;
		signal_t filterHPF(Channel& ch, const signal_t in) const;
		signal_t process(Channel& ch, const signal_t in);

		Channel left_;
		Channel right_;		

		Filter lpf_;
		Filter hpf_;

		// params
		// gain
		const float gain_db_ = 6.0;
		SmoothParam<float> wet_out_db_ = 0.4;

		// filter
		float filter_ = 0;
		float filter_cutoff_ = 0;
		float filter_res_ = 0;
		SmoothParam<float> filter_gain_comp_ = 1.0;
		FilterMode filter_mode_ = MODE_BANDPASS;

		// resampler
		float crush_ = 95;
		samples_t resample_hz_ = 33000;
		SmoothParam<float> per_sample_ = 2;

		// bitcrusher
		int bitdepth_ = 10;
		SmoothParam<float> bitscale_ = 1;
		SmoothParam<float> nuclear_ = 0;
		Mangler mangler_;

		//
		samples_t srate;

		void tick() {
			wet_out_db_.tick();
			per_sample_.tick();
			filter_gain_comp_.tick();
			bitscale_.tick();
			nuclear_.tick();
			lpf_.tick();
			hpf_.tick();
		}
};

/* end paranoia.hpp */

/* start paranoia.cpp */



// -------------------------------------------------------------------
// Internal data

void Paranoia::fixCrushParams() {
	bitdepth_ = (crush_ < 50.0f) ? 6 : 10;
	if (crush_ > 99.0f) {
		resample_hz_ = srate;
	} else if (crush_ > 50) {
		resample_hz_ = 300.0f + (crush_ - 50.0f) * 600.0f;
	} else {
		resample_hz_ = 300.0f + (50.0f - crush_) * 600.0f;
	}
	per_sample_ = (float) srate / (float) resample_hz_;
	bitscale_ = pow(2, bitdepth_ - 1) - 0.5f;
}

void Paranoia::fixFilterParams() {
	// cutoff shape is \/\/
	// need to compensate for filter gain
	filter_cutoff_ = 20.0f + fabs(fabs(160.0f - 3.2f * filter_) - 80.0f);
	filter_gain_comp_ = 3.0f - fabs(fabs(160.0f - 3.2f * filter_) - 80.0f) / 40.0f;

	// calc params from meta-param
	if (filter_ <= 80) {
		filter_mode_ = MODE_BANDPASS;
		filter_res_ = 10 + (filter_ / 8.0f);
	} else if (filter_ <= 99) {
		filter_res_ = 40.0;
		filter_gain_comp_ = 1;
		filter_mode_ = MODE_HPF;
	} else {
		filter_gain_comp_ = 1;
		filter_mode_ = MODE_OFF;
	}

	// set up R/C constants
	float lc = powf(0.5, 4.6 - (filter_cutoff_ / 27.2));
	lpf_.c = lc;
	float lr = powf(0.5, -0.6 + filter_res_ / 40.0);
	lpf_.one_minus_rc = 1.0 - (lr * lc);

	float hc = powf(0.5, 4.6 + (filter_cutoff_ / 34.8));
	hpf_.c = hc;
	float hr = powf(0.5, 3.0 - (filter_res_ / 43.5));
	hpf_.one_minus_rc = 1.0 - (hr * hc);
}

void Paranoia::updateParams() {
	// Update params if required.
	float wet_out_db = 30.0f * getParameterValue(PARAMETER_A) - 24.0f; // [-24,6]
	float crush = 100.0f * getParameterValue(PARAMETER_B); // [0, 100]
	float nuclear = 16.0f * getParameterValue(PARAMETER_C); // [0, 16]
	float filter = 100.0f * getParameterValue(PARAMETER_D); // [0, 100]

	if (fabs(wet_out_db - wet_out_db_.target()) > 0.1) {
		wet_out_db_ = wet_out_db;		
	}
	if (fabs(crush - crush_) > 0.1) {
		crush_ = crush;		
		fixCrushParams();  	
	}
	if (fabs(nuclear - nuclear_.target()) > 0.1) {
		nuclear_ = nuclear;		
	}
	if (fabs(filter - filter_) > 0.1) {
		filter_ = filter;		
		fixFilterParams();
	}
}

// TODO: Figure out a neat way to do this in stereo.
// BUG: mono only
void Paranoia::processAudio(AudioBuffer &buffer) {
	updateParams();

	// Process audio.  
	bool is_stereo = buffer.getChannels() > 1;
	FloatArray left_buf = buffer.getSamples(LEFT_CHANNEL);
	FloatArray right_buf;
	if (is_stereo) {
		right_buf = buffer.getSamples(RIGHT_CHANNEL);	
	}

	for (int i = 0; i < buffer.getSize(); ++i) {
		left_buf[i] = process(left_, left_buf[i]);
		if (is_stereo) {
			right_buf[i] = process(right_, right_buf[i]);		
		}
		tick();
	} 
}

signal_t Paranoia::process(Channel& ch, const signal_t in) {
	signal_t curr = in; // pregain(ch, in);
	curr = resample(ch, curr);
	curr = preSaturate(curr);
	curr = bitcrush(curr);

	if (filter_mode_ == MODE_LPF || filter_mode_ == MODE_BANDPASS) {
		curr = filterLPF(ch, curr);
	}
	if (filter_mode_ == MODE_HPF || filter_mode_ == MODE_BANDPASS) {
		curr = filterHPF(ch, curr);
	}
	curr = filter_gain_comp_ * DB_CO(wet_out_db_) * curr; // boost before post-saturate
	curr = postSaturate(curr);
	curr = ch.dc_filter.process(curr);
	return curr;
}

signal_t Paranoia::pregain(const Channel& ch, const signal_t in) const {
	return DB_CO(gain_db_) * in;
}

// resample is a dodgy resampler that sounds cool.

signal_t Paranoia::resample(Channel& ch, const signal_t in) const {
	ch.sample_csr += 1;
	if (ch.sample_csr < ch.next_sample && resample_hz_ < RESAMPLE_MAX) {
		return ch.prev_in;
	} else {
		ch.next_sample += per_sample_;
		if (resample_hz_ == RESAMPLE_MAX) {
			ch.sample_csr = ch.next_sample;
		}
		ch.prev_in = in;
		return in;
	}
}

signal_t Paranoia::bitcrush(const signal_t in) const {
	// boost from [-1, 1] to [0, 2^bitdepth) and truncate.
	float curr = (1.0 + in) * (float) bitscale_;

	// truncate
	curr = (int) curr;

	// Mangle (interpolating between L and R settings on mangle knob.
	float nuclear_l = (int) nuclear_;
	float mix = nuclear_ - nuclear_l;
	float nuclear_r = nuclear_l + ((mix > 0.001) ? 1 : 0);
	signal_t left = mangler_.mangleForBitDepth(nuclear_l, bitdepth_, curr);
	signal_t right = mangler_.mangleForBitDepth(nuclear_r, bitdepth_, curr);
	curr = left * (1.0 - mix) + right * mix;

	// Return to [-1, 1] range.
	curr = (curr / (float) bitscale_) - 1.0;
	float gain_l = mangler_.relgain(nuclear_l);
	float gain_r = mangler_.relgain(nuclear_r);
	float gain = gain_l * (1.0 - mix) + gain_r * mix;
	return curr * gain;
}

signal_t Paranoia::preSaturate(const signal_t in) const {
	signal_t curr = (1.0 + PRE_SHAPER) * in / (1.0 + PRE_SHAPER * fabs(in));
	return fmax(fmin(curr, CLAMP), -CLAMP);
}

signal_t Paranoia::postSaturate(const signal_t in) const {
	return (1.0 + POST_SHAPER)*in / (1.0 + POST_SHAPER * fabs(in));
}

// Applies a bandpass filter to the current sample.

float Paranoia::filterLPF(Channel& ch, const float in) const {
	ch.v0 = (lpf_.one_minus_rc) * ch.v0 + lpf_.c * (in - ch.v1);
	ch.v1 = (lpf_.one_minus_rc) * ch.v1 + lpf_.c * ch.v0;
	return ch.v1;
}

float Paranoia::filterHPF(Channel& ch, const float in) const {
	ch.hv0 = (hpf_.one_minus_rc) * ch.hv0 + hpf_.c * (in - ch.hv1);
	ch.hv1 = (hpf_.one_minus_rc) * ch.hv1 + hpf_.c * ch.hv0;
	return in - ch.hv1;
}

/* end paranoia.cpp */
