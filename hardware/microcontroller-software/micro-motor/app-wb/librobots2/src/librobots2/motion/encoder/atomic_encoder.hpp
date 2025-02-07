#ifndef ATOMIC_ENCODER_HPP
#define ATOMIC_ENCODER_HPP

#include <atomic>

typedef uint16_t(*getEncoderRaw)(void);

template<typename T, getEncoderRaw GER>
class AtomicEncoder{
public:
    AtomicEncoder(uint16_t encoder_resolution, bool inverted=false) :
        encoder_resolution_(encoder_resolution),
        inverted_(inverted)
    {
        lastEncoderValue_=GER();
        encoder_degrees_per_tick_ = T(360)/encoder_resolution_;
    };

    /**
    *   update, not protected by atomic
    */
    T update(){
        // retrieve raw encoder value
        const uint16_t value = GER();
        // cast difference to uint16 should handle overflow?
        diff_ = static_cast<int16_t>(value - lastEncoderValue_.load(std::memory_order_relaxed));
        if (not inverted_){
	        position_ += static_cast<int64_t>(diff_.load(std::memory_order_relaxed));
        }
        else{
            position_ -= static_cast<int64_t>(diff_.load(std::memory_order_relaxed));
        }
	    lastEncoderValue_ = value;

        // calculate angle in degrees
        angle_ = ( position_.load(std::memory_order_relaxed) % encoder_resolution_ ) * encoder_degrees_per_tick_;
        return angle_.load(std::memory_order_relaxed);
    };

    void reset(){
        lastEncoderValue_ = GER();
        diff_.store(0, std::memory_order_relaxed);
        position_.store(0, std::memory_order_relaxed);
        angle_.store(0, std::memory_order_relaxed);
    }

    inline
    T getAbsolutePosition(){
        return T(position_.load(std::memory_order_relaxed)) * encoder_degrees_per_tick_;
    };

    inline
    T getPosition(){
        return angle_.load(std::memory_order_relaxed);
    };

    inline
    T getDiff(){
        return diff_.load(std::memory_order_relaxed);
    }

private:

    std::atomic<uint16_t> lastEncoderValue_ = 0;
    std::atomic<int16_t> diff_ = 0;

    std::atomic<int32_t> position_ = 0;

    std::atomic<T> angle_ = 0;

    const uint16_t encoder_resolution_;
    const bool inverted_;
    T encoder_degrees_per_tick_;
};

#endif // ATOMIC_ENCODER_HPP
