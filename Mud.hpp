#include "Patch.h"


/* start util.hpp */

typedef int samples_t; // integral sample length or position
typedef float signal_t; // signal value

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
			//if (fabs(end - f) > 0.0001) {
				start = value;
				end = f;
				t = 0;
			//}
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


/* start mud.hpp */


// waveshapes
const float PRE_SHAPER = 0.5;
const float POST_SHAPER = 0.8;
const float CLAMP = 0.98;


class Mud : public Patch {
	public:

		struct Channel {
			public:

				// filter state
				float v0 = 0;
				float v1 = 0;
				float hv0 = 0;
				float hv1 = 0;

				// DC filter
				DcFilter dc_filter;

				void tick() {
					//
				}
		};

		struct Filter {
			SmoothParam<float, 128> c = 0.3;
			SmoothParam<float, 128> one_minus_rc = 0.98;

			void tick() {
				c.tick();
				one_minus_rc.tick();
			}
		};

		Mud() : Patch() {
			registerParameter(PARAMETER_A, "Mix");
			registerParameter(PARAMETER_B, "Filter");
			registerParameter(PARAMETER_C, "LFO Rate");
			registerParameter(PARAMETER_D, "LFO Depth");

			srate = getSampleRate();
		};

		void processAudio(AudioBuffer &buffer) override;

	private:
		void fixFilterParams();

		signal_t preSaturate(const signal_t in) const;
		signal_t postSaturate(const signal_t in) const;
		signal_t filterDC(Channel& ch, const signal_t in) const;
		signal_t filterLPF(Channel& ch, const signal_t in) const;
		signal_t filterHPF(Channel& ch, const signal_t in) const;
		signal_t process(Channel& ch, const signal_t in);

		Channel left_;
		Filter lpf_;
		Filter hpf_;

		// params
		// gain
		SmoothParam<float> mix_ = 1.0;

		// LFO
		float lfo_rate_ = 0;
		float lfo_depth_ = 0;		
		long lfo_counter_ = 0;
		float prv_filter_ = 0;

		// filter
		float filter_ = 50;
		float filter_cutoff_ = 0;
		float filter_res_ = 0;
		SmoothParam<float, 128> filter_gain_comp_ = 1.0;

		//
		samples_t srate;

		void tick() {
			mix_.tick();
			lpf_.tick();
			hpf_.tick();
			left_.tick();
			filter_gain_comp_.tick();
		}
};


// -------------------------------------------------------------------
// Internal data

void Mud::fixFilterParams() {
	lfo_counter_ += 1;

	float new_filter = filter_ + lfo_depth_ * sin(lfo_rate_ * lfo_counter_);
	new_filter = fmin(fmax(new_filter, 0.0f), 100.0f); // clamp
	new_filter = new_filter * 0.05f + prv_filter_ * 0.95f; // LERP to new filter value
	prv_filter_ = new_filter;

	// calc params from meta-param
	filter_res_ = 5.0 + ((int) new_filter / 2.0);
	filter_cutoff_ = 5.0 + fabs(fabs(160.0 - 3.2 * new_filter) - 80.0);
	filter_gain_comp_ = 3.0 - fabs(fabs(160.0 - 3.2 * new_filter) - 80.0) / 40.0;

	// set up R/C constants
	float lc = powf(0.5, 4.6 - (filter_cutoff_ / 27.2));
	lpf_.c = lc;
	float lr = powf(0.5, -0.6 + filter_res_ / 40.0);
	lpf_.one_minus_rc = 1.0 - (lr * lc);

	float hc = powf(0.5, 4.6 + (filter_cutoff_ / 34.8));
	hpf_.c = hc;
	float hr = powf(0.5, 3.0 - (filter_res_ / 63.5));
	hpf_.one_minus_rc = 1.0 - (hr * hc);
}


/**
  Run/process function for plugins without MIDI input.
  */
void Mud::processAudio(AudioBuffer &buffer) {
	// TODO(dca): This is unsafe if mono buffers provided.
	FloatArray left_buf = buffer.getSamples(LEFT_CHANNEL);

	// Update parameters
	mix_ = getParameterValue(PARAMETER_A); 
	filter_ = 100.0f * getParameterValue(PARAMETER_B);
	lfo_rate_ = 0.01f * getParameterValue(PARAMETER_C);
	lfo_depth_ = 100.0f * getParameterValue(PARAMETER_D);

	for (int i = 0; i < buffer.getSize(); ++i) {
	    
	    if (i % 16 == 0) {
	        fixFilterParams();
	    }
	    	
		left_buf[i] = process(left_, left_buf[i]);
		tick();
	}
}

signal_t Mud::process(Channel& ch, const signal_t in) {
	signal_t curr = preSaturate(in);
	curr = filterLPF(ch, curr);
	curr = filterHPF(ch, curr);
	curr = postSaturate(curr);
	curr = ch.dc_filter.process(curr);

	if (mix_ < 0.5) {
		// dry full vol, fade in wet
		return in + 2.0 * mix_ * curr;
	} else {
		// wet full vol, fade out dry
		return curr + 2.0 * (1.0 - mix_) * in;
	}
}

signal_t Mud::preSaturate(const signal_t in) const {
	signal_t curr = (1.0 + PRE_SHAPER) * in / (1.0 + PRE_SHAPER * fabs(in));
	return fmax(fmin(curr, CLAMP), -CLAMP);
}

signal_t Mud::postSaturate(const signal_t in) const {
	return (1.0 + POST_SHAPER)*in / (1.0 + POST_SHAPER * fabs(in));
}

// Applies a bandpass filter to the current sample.

float Mud::filterLPF(Channel& ch, const float in) const {
	ch.v0 = (lpf_.one_minus_rc) * ch.v0 + lpf_.c * (in - ch.v1);
	ch.v1 = (lpf_.one_minus_rc) * ch.v1 + lpf_.c * ch.v0;
	return ch.v1;
}

float Mud::filterHPF(Channel& ch, const float in) const {
	ch.hv0 = (hpf_.one_minus_rc) * ch.hv0 + hpf_.c * (in - ch.hv1);
	ch.hv1 = (hpf_.one_minus_rc) * ch.hv1 + hpf_.c * ch.hv0;
	return in - ch.hv1;
}


/* end mud.hpp */
