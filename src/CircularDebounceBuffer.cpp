#include "CircularDebounceBuffer.h"

/**
* @brief Constructor for CircularDebounceBuffer.
*
* Initializes the debouncer with the given parameters. The buffer is cleared, and flags are reset.
* The default threshold is 90%, and callbacks are empty.
*
* @param id Arbitrary identifier (unused in zero-arg callbacks but available for extensions).
* @param pin The digital pin number to read for button state.
* @param isActiveLow If true, a LOW reading indicates "pressed" (e.g., pull-up wiring).
* @param delayBetweenUs Microseconds between each sample (default: 1000us = 1ms).
*/
CircularDebounceBuffer::CircularDebounceBuffer(
    uint8_t id,
    uint8_t pin,
    bool    isActiveLow,
    uint32_t delayBetweenUs
)
: _pin_ID(id),
  _pin(pin),
  _isActiveLow(isActiveLow),
  _debouncing(false),
  _stableState(false),
  _pressedDetected(false),
  _head(0),
  _thresholdPercentage(90),                             // default to 90% (you can override in setup)
  _callbackCounter(0),
  _delayBetweenSamples(delayBetweenUs),                 // initialize Delay with desired interval
  _lastGestureTimeUs(0),
  _longPressEnable(false),
  _longPressCallbackCounter(0),
  _doublePressCallbackCounter(0),
  _longPressDurationMs(LONG_PRESS_DURATION_MS_DEFAULT),
  _pressConfirmTimeUs(0),                               // Initialize to 0
  _longPressFiredThisDebounceSession(false),            // to ensure long-press callback fires only once per press
  _doublePressFiredThisSession(false),
  _doublePressEnable(false),
  _doublePressMaxWindowMs(DOUBLE_PRESS_WINDOW_MS),
  _lastShortReleaseTimeUs(0),
  _waitingForSecondPress(false),
  _pendingShortPress(false)
{
    pinMode(_pin, _isActiveLow ? INPUT_PULLUP : INPUT);
    clearBuffer();
}

/**
 * @brief Clears the entire sample buffer (sets all slots to false) and resets head to 0.
 */
void CircularDebounceBuffer::clearBuffer()
{
    for (size_t i = 0; i < BUFFER_SIZE; ++i)
    {
        _buffer[i] = false;
    }
    _head = 0;
}

/**
 * @brief Sets the percentage of BUFFER_SIZE that must be "true" to confirm a press.
 *
 * Calculates threshold as ceil(BUFFER_SIZE * percentage / 100). Ignores if percentage > 100.
 *
 * @param percentage The required agreement percentage (e.g., 60 for 60%).
 */
void CircularDebounceBuffer::setThreshold(uint8_t percentage)
{
    if (percentage <= 100)
    {
        _thresholdPercentage = percentage;
    }
}

/**
 * @brief Adjusts the sample interval (in microseconds).
 *
 * Updates the Delay object's target time without restarting the timer.
 *
 * @param newDelayUs New interval in microseconds.
 */
void CircularDebounceBuffer::setSampleIntervalUs(uint32_t newDelayUs)
{
    _delayBetweenSamples.updateDelayTime(newDelayUs);
}

/**
 * @brief Enable long-press detection with specified duration.
 * 
 * @param durationMs - Specific duration in milliseconds to consider a long press
 */
void CircularDebounceBuffer::enableLongPress(uint32_t durationMs)
{
    if(durationMs == 0) return; // invalid duration

    if(_longPressEnable && (_longPressDurationMs == durationMs))
    {
        // Already enabled with the same duration, do nothing
        return;
    }

    _longPressDurationMs = durationMs;
    _longPressEnable = true;
    _longPressFiredThisDebounceSession = false;
    
    // Clear old callbacks related to long press
    avr_algorithms::for_index_n(MAX_CALLBACKS,[&](size_t i){
        _longPressCallbacks[i] = CbSlot{};                      //  Reset to default state
    });

    // Finally reset long press callback counter    
    _longPressCallbackCounter = 0;
}

/**
 * @brief Disable long-press detection and reset state.
 * 
 */
void CircularDebounceBuffer::disableLongPress()
{
    _longPressEnable = false;
    _longPressCallbackCounter = 0;
    _longPressFiredThisDebounceSession = false;

    // Clear all callbacks related to long press
    avr_algorithms::for_index_n(MAX_CALLBACKS,[&](size_t i){
        _longPressCallbacks[i] = CbSlot{};                      //  Reset to default state
    });
}

/**
 * @brief Enable double-press detection with specified max time window in ms
 * 
 * @param maxWindowMs - Max window between short pressed in milliseconds to consider a double press
 */
void CircularDebounceBuffer::enableDoublePress(uint32_t maxWindowMs)
{
    if(maxWindowMs == 0) return; // invalid duration

    if(_doublePressEnable && (_doublePressMaxWindowMs == maxWindowMs))
    {
        // Already enabled with the same duration, do nothing
        return;
    }

    _doublePressMaxWindowMs = maxWindowMs;
    _doublePressEnable = true;
    _waitingForSecondPress = false;
    
    // Clear old callbacks related to double press
    avr_algorithms::for_index_n(MAX_CALLBACKS,[&](size_t i){
        _doublePressCallbacks[i] = CbSlot{};                      //  Reset to default state
    });

    // Finally reset double press callback counter    
    _doublePressCallbackCounter = 0;
}

/**
 * @brief Disable double-press detection and reset state.
 * 
 */
void CircularDebounceBuffer::disableDoublePress()
{
    _doublePressEnable = false;
    _doublePressCallbackCounter = 0;
    _waitingForSecondPress = false;

    // Clear all callbacks related to double press
    avr_algorithms::for_index_n(MAX_CALLBACKS,[&](size_t i){
        _doublePressCallbacks[i] = CbSlot{};                      //  Reset to default state
    });
}


/**
 * @brief Registers a zero-argument callback. Up to MAX_CALLBACKS can be stored.
 *
 * @param cb The callback function to add.
 */
void CircularDebounceBuffer::addCallback(Callback cb)
{
    // Validate input, if null or exceed max, return
    if(!cb || _callbackCounter >= MAX_CALLBACKS) return;

    // Add the Plain callback
    auto& slot = _callbacks[_callbackCounter++];
    slot.kind = CallbackKind::Plain;
    slot.fn.plain = cb;
    slot.ctx = nullptr;
}

/**
 * @brief Register a extended callback with context
 * 
 * @param cb - callback kind
 * @param ctx - user data that it pass to the callback 
 */
void CircularDebounceBuffer::addCallbackEx(CallbackEx cb, void* ctx)
{
    // Validate input
    if(!cb || _callbackCounter >= MAX_CALLBACKS) return;

    // Add the Extended callback
    auto& slot =_callbacks[_callbackCounter++];
    slot.kind = CallbackKind::WithCtx;
    slot.fn.ex = cb;
    slot.ctx   = ctx;
}

/**
 * @brief Register a long-press callback (plain)
 * 
 * @param cb - LongPressCallback function pointer
 */
void CircularDebounceBuffer::addLongPressCallback(LongPressCallback cb)
{
    // Validate input
    if(!cb || _longPressCallbackCounter >= MAX_CALLBACKS) return;

    // Add the Long Press Plain callback
    auto& slot = _longPressCallbacks[_longPressCallbackCounter++];
    slot.kind = CallbackKind::Plain;
    slot.fn.plain = (Callback)cb;
}

/**
 * @brief Register a long-press callback with context
 * 
 * @param cb - LongPressCallbackEx function pointer
 * @param ctx - user data that it pass to the callback 
 */
void CircularDebounceBuffer::addLongPressCallbackEx(LongPressCallbackEx cb, void* ctx)
{
    // Validate input
    if(!cb || _longPressCallbackCounter >= MAX_CALLBACKS) return;

    // Add the Long Press Extended callback
    auto& slot = _longPressCallbacks[_longPressCallbackCounter++];
    slot.kind = CallbackKind::WithCtx;
    slot.fn.ex = (CallbackEx)cb;
    slot.ctx   = ctx;
}

/**
 * @brief Register a double-press callback (plain) that will be executed once a double press is confirmed.
 * 
 * @param cb - DoublePressCallback function pointer
 */
void CircularDebounceBuffer::addDoublePressCallback(DoublePressCallback cb)
{
    // Validate input
    if(!cb || _doublePressCallbackCounter >= MAX_CALLBACKS) return;

    // Add the Double Press Plain callback
    auto& slot = _doublePressCallbacks[_doublePressCallbackCounter++];
    slot.kind = CallbackKind::Plain;
    slot.fn.plain = (Callback)cb;
}

void CircularDebounceBuffer::addDoublePressCallbackEx(DoublePressCallbackEx cb, void* ctx)
{
    // Validate input
    if(!cb || _doublePressCallbackCounter >= MAX_CALLBACKS) return;

    // Add the Double Press Extended callback
    auto& slot  = _doublePressCallbacks[_doublePressCallbackCounter++];
    slot.kind = CallbackKind::WithCtx;
    slot.fn.ex = (CallbackEx)cb;
    slot.ctx   = ctx;    
}



/** @brief Arms the debouncer for a new press, called from a raw FALLING-edge ISR.
 * 
 * @details
 *  Called from your raw FALLING‐edge ISR. Arms the debouncer for a new press.
 * It will clear the buffer, reset flags, and start the Delay countdown.
 */
void CircularDebounceBuffer::startDebounce()
{
    // Only arm if we’re not already debouncing AND the last stable state is “not pressed.”
    if (!_debouncing && !_stableState) 
    {
        _debouncing      = true;
        _pressedDetected = false;                   // allow the next “press” to fire a callback
        clearBuffer();                              // drop any old samples
        _delayBetweenSamples.restartTimer();        // begin counting from now
    }
}

/**
 * @brief Advance the debouncer state machine by one sample.
 *
 * @details
 * This method should be called from the main loop at a relatively high rate.
 * Internally it limits its own work using a per-sample delay
 * (`_delayBetweenSamples`) so it only takes a sample every N microseconds.
 *
 * **What it does at each call (when the sample delay elapses):**
 *
 *  1) **Sample input**:
 *     - Reads the raw digital pin ('digitalRead(_pin)'), then applies the
 *       active-low option (`_isActiveLow ? !raw : raw`) to produce a logical
 *       “pressed” bit.
 *     - Pushes the bit into a circular buffer '_buffer' of size 'BUFFER_SIZE'
 *       and advances '_head' modulo the buffer size.
 *
 *  2) **Consensus debounce**:
 *     - Counts how many "true" bits are in the buffer ("trueCount").
 *     - Computes a consensus threshold:
 *         'thresholdCount = ceil(BUFFER_SIZE * _thresholdPercentage / 100)'
 *       This creates **hysteresis**: we consider the input “pressed” if
 *       'trueCount >= thresholdCount', and “released” if
 *       'trueCount <= (BUFFER_SIZE - thresholdCount)'.
 *       Anything in-between is considered unstable and we keep the prior state.
 *
 *  3) **State machine**:
 *     - If we have **not** confirmed a press yet (!_pressedDetected):
 *         * When consensus crosses the **press** threshold, we set:
 *             '_stableState=true; _pressedDetected=true'.
 *         * If **long-press** is enabled:
 *             - Remember the moment the press was confirmed ('_pressConfirmTimeUs = micros()').
 *             - Arm the long-press path ('_longPressFiredThisDebounceSession=false').
 *         * Else if **double-press** is enabled and we were already waiting for a
 *           second press ('_waitingForSecondPress==true'):
 *             - If the waiting window has already expired, we cancel the double-press arming.
 *         * Else (normal short-press behavior): **fire short-press immediately on press**
 *           via 'fireAll()'.
 *           to the release branch and remove it here.)
 *
 *     - If we are currently **pressed** (`_pressedDetected==true`):
 *         * If **long-press** is enabled and not yet fired:
 *             - Check elapsed time since `_pressConfirmTimeUs`.
 *             - When `elapsed >= _longPressDurationMs`, fire `fireAllLongPress(holdMs)`,
 *               mark `_longPressFiredThisDebounceSession=true`, and cancel double-press
 *               arming (`_waitingForSecondPress=false`) because a long-press consumes
 *               the gesture.
 *
 *     - If consensus crosses the **release** threshold:
 *         * We finalize the cycle:
 *             `_stableState=false; _pressedDetected=false; _debouncing=false; clearBuffer();`
 *         * **Double-press** logic (only for short presses):
 *             - If `_doublePressEnable` and **no** long-press was fired:
 *                 + If we were **waitingForSecondPress** and the elapsed time since
 *                   the prior **short release** is within `_doublePressMaxWindowMs`,
 *                   fire `fireAllDoublePress()` and clear waiting.
 *               
 *
 * Notes:
 *  - Time arithmetic uses 'micros()' unsigned wrap-safe subtraction.
 *  - The buffer is fully cleared on release to avoid residual bits influencing
 *    the next cycle’s consensus.
 *  - The method is **not ISR-safe**; call it from the main loop/task, not an ISR.
 *  - Long-press and double-press are mutually exclusive per cycle: a long-press
 *    cancels any pending double-press.

 * @warning Must be called every loop(). Non-blocking.
 */
void CircularDebounceBuffer::update()
{
    // === 1. VALIDATE GESTURE SESSION TIMEOUT ===
    // Reset all gesture states after the session timeout has expired without any activity
    const uint32_t now = micros();
    if (_lastGestureTimeUs != 0 && 
        (now - _lastGestureTimeUs) > (GESTURE_SESSION_TIMEOUT_MS * 1000UL)) 
    {
        _waitingForSecondPress = false;
        _pendingShortPress = false;
    }

    // TIMEOUT: only when not debouncing (idle)
    if (!_debouncing && _doublePressEnable && _waitingForSecondPress && _pendingShortPress) {
        if (now - _lastShortReleaseTimeUs > (_doublePressMaxWindowMs * 1000UL)) {
            fireAll();
            _pendingShortPress = false;
            _waitingForSecondPress = false;
        }
    }


    // Validate if we are on an active debounce Session, if not return 
    if (!_debouncing) return;

    // Only proceed if the per-sample delay has elapse (rate-limit sample) 
    if (!_delayBetweenSamples.isDelayTimeElapsed()) return;


    // === 2. START DEBOUNCE: SAMPLE INPUT === 
    // Take sample
    bool raw      = digitalRead(_pin);
    bool adjusted = _isActiveLow ? !raw : raw;   // Normalize pressed = "true"
    _buffer[_head] = adjusted;
    _head = (size_t)((_head + 1) % BUFFER_SIZE);

    // Count consensus: How many "true" bits are in the buffer now
    uint8_t trueCount = 0;
    for (size_t i = 0; i < BUFFER_SIZE; ++i) 
        if (_buffer[i]) ++trueCount;

    // Compute the absolute number of “true” samples needed:
    // thresholdCount = ceiling(BUFFER_SIZE * thresholdPercentage / 100).
    const uint8_t thresholdCount = (BUFFER_SIZE * _thresholdPercentage + 99) / 100;

    // =========================

    // Hysteresis band:
    //   - Pressed if trueCount >= thresholdCount
    //   - Released if trueCount <= (BUFFER_SIZE - thresholdCount)
    //   - Otherwise unchanged (still bouncing / indeterminate)
  
    // === 3. HANDLE PRESS AFTER CONFIRMATION ===
    // A) Confirm press (if not yet confirmed)
    if (!_pressedDetected) {
        
        if (trueCount >= thresholdCount) {

            // we have reached enough consecutives "true" samples -> Confirm "pressed"
            _stableState     = true;
            _pressedDetected = true;

            // Start timer for new gesture activity
            _lastGestureTimeUs = now;
            
            // If long Press detection is Enable: Arm LongPress detection session
            if (_longPressEnable) 
            {   
              // Start measuring hold time for long-press
              _pressConfirmTimeUs = micros();
              _longPressFiredThisDebounceSession = false;
            }
            
            // A.1) Check for possible second Press detection( If second press is detected within window) from previouse session
            if (_doublePressEnable && _waitingForSecondPress) 
            {             
              // If we exceeded the window, cancel double-press arming
              uint32_t dt = micros() - _lastShortReleaseTimeUs; 
              if ( dt > (_doublePressMaxWindowMs * 1000UL)) 
              {        
                _waitingForSecondPress = false;
                _doublePressFiredThisSession = false;
              }
            }
            else 
            {
                // On press: if it is the first press on the session, let short press on prending procces until double press gesture is discard 
                _pendingShortPress =true;
            }
        }
        return; // Until a press is confirm, nothing to do until next pass
    }

    // ======================


    // === 4. HANDLE HOLD BUTTON LONG PRESS ===
    // B) While pressed: So not release detected -> check long-press timeout (if enable)
    if (_longPressEnable && !_longPressFiredThisDebounceSession && _pressConfirmTimeUs != 0) {        

        const uint32_t elapsed = micros() - _pressConfirmTimeUs;
        if (elapsed >= (_longPressDurationMs * 1000UL)) 
        {
           const uint32_t holdMs = elapsed / 1000UL;         
           fireAllLongPress(holdMs);
           _longPressFiredThisDebounceSession = true;
           _waitingForSecondPress = false; // long-press cancels double-press logic
        }
    }
    // ===========================

    // === 5. HANDLE RELEASE DETECTION ===
    // C) Check release consensus: If a press has already been detected (_pressedDetected == true):
    //    Wait until we see enough “false” samples to confirm a release.
    if (trueCount <= (BUFFER_SIZE - thresholdCount)) {
        _stableState     = false;
        _pressedDetected = false;
        _debouncing      = false;
        clearBuffer();                  // Clear history to avoid residual bits from influencing next cycle

        // === DOUBLE PRESS LOGIC ===
        // Double-press handling (only if no long-press was fired this cycle)
        if (_doublePressEnable && !_longPressFiredThisDebounceSession) {
            if (_waitingForSecondPress) 
            {
                uint32_t dt = now - _lastShortReleaseTimeUs;
                if (dt <= (_doublePressMaxWindowMs * 1000UL)) 
                {
                    fireAllDoublePress();
                    _doublePressFiredThisSession = true;
                }
                _waitingForSecondPress = false;
            } 
            else
            {
                _lastShortReleaseTimeUs = now;
                _waitingForSecondPress = true;
            }
        }
        else
        {
            // === SHORT PRESS: Only if not long-press and not double-press ===
            if (!_longPressFiredThisDebounceSession && !_doublePressFiredThisSession) {
                fireAll();
                _pendingShortPress = false;
            }
        }       
        return;
    }

    // ======================================================
    // D) If we get here, it means trueCount is between thresholdCount and
    //    (BUFFER_SIZE - thresholdCount). Still bouncing, so just wait.
    // ======================================================
}

/**
 * @brief Gets the current stable debounced state.
 *
 * @return The stable state (true = pressed).
 */
bool CircularDebounceBuffer::getStableState() const
{
    return _stableState;
}

/**@brief Completely reinitialize the debouncer 
 * Completely re‐initialize:
 *  • Clear the buffer
 *  • Reset flags (_stableState, _pressedDetected, _debouncing)
 *  • Reset callback counter
 *  • Restart the internal Delay timer
 */
void CircularDebounceBuffer::reset()
{
    clearBuffer();
    _stableState     = false;
    _pressedDetected = false;
    _debouncing      = false;
    _callbackCounter = 0;
}


/**
 * @brief Execute all-register callbacks( once per confirm press).
 * 
 */
void CircularDebounceBuffer::fireAll()
{
   avr_algorithms::for_each_element(_callbacks, _callbackCounter, [&](const CbSlot& cb_slot){   
        if(cb_slot.kind == CallbackKind::WithCtx)
        {
            if(cb_slot.fn.ex)          
                cb_slot.fn.ex(cb_slot.ctx);          
        }else
        {
            if(cb_slot.fn.plain)
            cb_slot.fn.plain();
        }
   });
}


/**
 * @brief Fire all registered long-press callbacks
 * 
 * @details 
 * This method iterates through all registered long-press callbacks and invokes them with the provided hold duration.
 * 
 * @param holdDurationMs - duration in milliseconds the button was held
 */
void CircularDebounceBuffer::fireAllLongPress(uint32_t holdDurationMs)
{

    avr_algorithms::for_each_element(_longPressCallbacks, _longPressCallbackCounter, [&holdDurationMs](const CbSlot& slot){   
                 
        // Invoke based on kind
        if (slot.kind == CallbackKind::WithCtx && slot.fn.ex) {
            ((LongPressCallbackEx)slot.fn.ex)(slot.ctx, holdDurationMs);    // pass context
        } else if (slot.fn.plain) {
            ((LongPressCallback)slot.fn.plain)(holdDurationMs);             // plain callback
        }
    });
}

/**
 * @brief Fire all Double-press register callbacks 
 * 
 */
void CircularDebounceBuffer::fireAllDoublePress()
{
    avr_algorithms::for_each_element(_doublePressCallbacks, _doublePressCallbackCounter, [&](const CbSlot& slot){   
       // Invoke bases on kind
       if(slot.kind == CallbackKind::WithCtx && slot.fn.ex)
       {
           ((DoublePressCallbackEx)slot.fn.ex)(slot.ctx);
        }else if(slot.fn.plain)
        {
           ((DoublePressCallback)slot.fn.plain)();
        }
    });
}