/**
 * @file CircularDebounceBuffer.h
 * @brief Header file for the CircularDebounceBuffer class, providing a robust debounce mechanism for digital button inputs.
 * Advanced interrupt-triggered, non-blocking button debouncer using a circular buffer and a consensus voting.
 *
 * This class implements a circular buffer-based debouncing algorithm for handling noisy button presses on micro-controllers like Arduino.
 * It collects periodic samples of the pin state and uses a consensus threshold to confirm presses and releases, making it resilient to bounce.
 * Debouncing is triggered by a FALLING edge interrupt and performed non-blockingly in the main loop.
 * It uses a fixed-size circular buffer to store recent samples and confirms state changes only
 * when a configurable percentage of samples agree (consensus threshold).
 * 
 * Key features:
 * - Triggered by FALLING edge interrupt (optimizes CPU usage when idle)
 * - Non-blocking operation — 'update()' is called from 'loop()'
 * - Supports active-low (pull-up) and active-high (pull-down) wiring
 * - Short Press, long Press(immediate trigger callbacks after detection) and Double-pressed detection
 * - Configurable sample interval and debounce threshold
 * - Zero arguments and context-aware callbacks
 * - Symmetric press/release detection using inverted threshold
 * - Supports both zero-argument callbacks and context-aware callbacks (with user data)
 *
 * @author David Soriano Artal
 * @date 2025
 * @version 1.0.0
 * 
 */
#pragma once

#include "HardwareSerial.h"
#include <Arduino.h>
#include <stdint.h>
#include "Delay.h"   
#include <stdarg.h>
#include "avr_algorithms.h"

// Session timeout for gesture recognition.
static constexpr uint32_t GESTURE_SESSION_TIMEOUT_MS = 3500;                // 1.5 second of silence = new session

// Default long-press duration = 2000ms
static constexpr uint32_t LONG_PRESS_DURATION_MS_DEFAULT = 3000 ; 

// Default window for double-press = 500ms
static constexpr uint32_t DOUBLE_PRESS_WINDOW_MS = 500;

// How many samples we keep in the circular buffer (16 is typical for ~15-20ms windows.)
static constexpr size_t BUFFER_SIZE = 16;

// Maximum number of zero‐arg callbacks we support
static constexpr size_t MAX_CALLBACKS = 4;

// === CALLBACKS TYPES ===
using Callback = void(*)();                                             // Plain C function pointer
using CallbackPlain = void(*)();                                        // alias
using CallbackEx = void(*)(void*);                                      // Extended callback that accepts a function pointer + user data
using LongPressCallback = void(*)(uint32_t holdDurationMs);             // Long press callback with duration in ms
using LongPressCallbackEx = void(*)(void*, uint32_t holdDurationMs);    // Extended long press callback with context
using DoublePressCallback = void(*)();                                  // Double press callback with no arguments
using DoublePressCallbackEx = void(*)(void*);                           // Extended double press callback with context
// ========================



/**
 * @class CircularDebounceBuffer
 * @brief A class for debouncing digital button inputs using a circular buffer and consensus threshold.
 *
 * This class provides an advanced debouncing solution that samples the button pin at regular intervals,
 * stores the last BUFFER_SIZE samples in a circular buffer, and confirms a press or release based on
 * a configurable percentage threshold of agreeing samples. It supports active-low or active-high wiring,
 * adjustable sample intervals, and zero-argument callbacks for press events.
 * 
 * @note Alows normal, double and long Press detection.
 *
 * @note designed for mechanical switches with bounce durations <~20ms
 * @note Debouncing is armed only on FALLING edges to optimize idle performance.
 * @note Release detection uses an inverted threshold for symmetry
 *
 *  @note The constructor automatically configures pinMode():
 *       - INPUT_PULLUP if isActiveLow == true (recommended)
 *       - INPUT if isActiveLow == false
 * 
 * Usage:
 * - Instantiate with pin details.
 * - Set threshold and interval as needed.
 * - Register callbacks for press events.
 * - Attach an interrupt to call startDebounce() on FALLING edge.
 * - Call update() repeatedly in the main loop.
 *
 * @example Basic usage(active-low button with internal pull-up):
 * Usage:
 * @code
 * 
 * CircularDebounceBuffer btn(1, 2); // pin 2, active-low
 *
 * void setup() {
 *   btn.addCallback([] { Serial.println("Short press"); });
 *   btn.enableLongPress(1000);
 *   btn.addLongPressCallback([](uint32_t ms) { Serial.printf("Long press: %lums\n", ms); });
 *   btn.enableDoublePress(400);
 *   btn.addDoublePressCallback([] { Serial.println("DOUBLE PRESS!"); });
 *
 *   attachInterrupt(digitalPinToInterrupt(2), [] { btn.startDebounce(); }, FALLING);
 * }
 *
 * void loop() {
 *   btn.update();
 * }
 * @endcode 
 */
class CircularDebounceBuffer {
public:
    /**
     * @param id                An arbitrary ID (not used by zero‐arg callbacks)
     * @param pin               The Arduino digital pin to debounce
     * @param isActiveLow       If true, a LOW read means “pressed”
     * @param delayBetweenUs    How many microseconds between each raw sample
     */
    CircularDebounceBuffer(
        uint8_t id,
        uint8_t pin,
        bool    isActiveLow = true,
        uint32_t delayBetweenUs = 1000
    );

    // --- PUBLIC CALLBACK REGISTRATION API ---
    
    /**
     * @brief Register a zero-argument callback to be called once per confirmed press.
     *
     * @param cb Function pointer to callback. Must not be nullptr.
     */
    void addCallback(Callback cb);

    /**
     * @brief Register a callback with user context (e.g., for lambdas capturing `this`).
     *
     * @param cb  Callback function accepting a void* context
     * @param ctx User data passed to the callback when invoked
     */
    void addCallbackEx(CallbackEx cb, void* ctx);

    //  ================================

    // === PUBLIC LONG PRESS SUPPORT API ===

    /**
     * @brief Enable long-press detection
     * 
     * @param durationMs - Specific duration in milliseconds to consider a long press
     */
    void enableLongPress(uint32_t durationMs);                 
    
    /**
     * @brief Disable long-press detection
     * 
     */
    void disableLongPress();                                   
    
    /**
     * @brief Add a Plain long-press callback to be called once per confirmed long press.
     * 
     * @param cb - LongPressCallback function pointer
     */
    void addLongPressCallback(LongPressCallback cb);

    /**
     * @brief Add a context-aware long-press callback to be called once per confirmed long press.
     * 
     * @param cb - LongPressCallbackEx function pointer
     * @param ctx - user data that it pass to the callback 
     */
    void addLongPressCallbackEx(LongPressCallbackEx cb, void* ctx);
    // ================================

    // ===  PUBLIC DOUBLE PRESS SUPPORT API ===

    /**
     * @brief Enable double-press detection with specified max time window in ms
     * 
     * @param maxWindowMs - Max window between short pressed in milliseconds to consider a double press
     */
    void enableDoublePress(uint32_t maxWindowMs);                 

    /**
     * @brief Disable double-press detection
     * 
     */
    void disableDoublePress();

    /**
     * @brief Add a Plain double-press callback to be called once per confirmed double press.
     * 
     * @param cb - DoublePressCallback function pointer
     */
    void addDoublePressCallback(DoublePressCallback cb);

    /**
     * @brief Add a context-aware double-press callback to be called once per confirmed double press.
     * 
     * @param cb - DoublePressCallbackEx function pointer
     * @param ctx - user data that it pass to the callback 
     */
    void addDoublePressCallbackEx(DoublePressCallbackEx cb, void* ctx);

    // ================================

    // === PUBLIC API ===
    /**
     * @brief Set the consensus threshold for confirming a press.
     *
     * The button is considered pressed when at least this percentage of samples in the buffer
     * are "true" (pressed). Release uses symmetric logic: ≤ (100% - threshold).
     *
     * @param percentage Value from 1–100. Values >100 are ignored. Default: 90%.
     *
     * @note Uses ceiling division: thresholdCount = ceil(BUFFER_SIZE * percentage / 100)
    */
    void setThreshold(uint8_t percentage);

    /**
     * @brief Change the sampling interval.
     *
     * @param newDelayUs New interval between samples in microseconds.
    */
    void setSampleIntervalUs(uint32_t newDelayUs);

    /**
     * @brief Must be called frequently from loop(). Performs timed sampling and state evaluation.
     *
     * This method is non-blocking and only takes action when the sample interval has elapsed.
     */
    void update();

    /**
     * @brief Call from a FALLING edge interrupt to start a new debounce cycle.
     *
     * Only arms debouncing if currently idle and stable state is released.
     * Safe to call repeatedly — ignored if already active.
     */
    void startDebounce();

    /**
     * @brief Get the current debounced stable state.
     *
     * @return true if button is confirmed pressed, false if released.
     */
    bool getStableState() const;

    /**
     * @brief Reset the debouncer to initial power-on state.
     *
     * Clears buffer, resets all flags and timer. Call if reconfiguring pin or recovering from error.
     */
    void reset();

    // ================================

private:

    // --- callback slot that can hold either kind of callbacks
    enum class CallbackKind: uint8_t { Plain  , WithCtx};

    /**
     * @brief Represents a single callback "slot".
     * 
     * @details Description
     * This struct allows us to register either:
     *   - A plain C-style callback:    void(*)()
     *   - A callback with context:     void(*)(void*)
     *
     * The 'kind' field tells which variant is active.
     * The union 'fn' stores the actual function pointer.
     * The optional 'ctx' pointer holds user data (e.g. 'this'),
     *   but is only valid when 'kind == CbKind::WithCtx'.
     *  
     * @note 
     *  Each instance can represent either:
     *      - A plain function pointer
     *      - An extended function pointer that used context pointer 
     * @example
     * Usage:
     *   CbSlot slot;
     *
     *   // Plain function
     *   slot.kind = CbKind::Plain;
     *   slot.fn.plain = someFunction;
     *
     *   // Function with context
     *   slot.kind = CbKind::WithCtx;
     *   slot.fn.ex = someFunctionWithCtx;
     *   slot.ctx   = this;
     *
     * When invoking:
     *   if (slot.kind == CbKind::Plain) {
     *       slot.fn.plain();
     *   } else {
     *       slot.fn.ex(slot.ctx);
     *   }
     */
    struct CbSlot
    {
        CallbackKind kind = CallbackKind::Plain; // Tag that indicates which type of callback is stored
        union 
        {
            CallbackPlain plain;    // plain callback, with no context
            CallbackEx  ex;         // extended callback, with context
        }fn{nullptr};               // union initialization to nullptr, so it initializes the 'plain' member with nullptr
        void* ctx = nullptr;    // only used when kind == withCtx       
    };

    uint8_t   _pin_ID;                     // arbitrary ID (unused in zero‐arg callback)
    uint8_t   _pin;                        // the Arduino pin number
    bool      _isActiveLow;                // if true, LOW means “pressed”
    bool      _debouncing;                 // true while we’re in a press/release cycle
    bool      _stableState;                // last confirmed stable state (true=pressed)
    bool      _pressedDetected;            // set true once we have fired the press callback
    bool      _buffer[BUFFER_SIZE];        // circular buffer of last BUFFER_SIZE samples
    size_t    _head;                       // index of next slot to overwrite
    uint8_t   _thresholdPercentage;        // e.g. 60 means 60% of BUFFER_SIZE
    
    // --- Callback management ---
    CbSlot  _callbacks[MAX_CALLBACKS];   // array of registered callbacks
    size_t  _callbackCounter;            // how many callbacks are registered

    CbSlot  _longPressCallbacks[MAX_CALLBACKS];   // array of registered long-press callbacks
    size_t  _longPressCallbackCounter;            // how many long-press callbacks are registered

    CbSlot  _doublePressCallbacks[MAX_CALLBACKS];   // array of registered double-press callbacks
    size_t  _doublePressCallbackCounter;            // how many double-press callbacks are

    // --- Timing management ---
    Delay     _delayBetweenSamples;         // Delay object (micros‐based)

    // === GESTURE RECOGNITION API ===
    uint32_t _lastGestureTimeUs;            // Tracks time since the last time that a gesture recognition session started

    // === LONG PRESS SUPPORT API ===
    
    //--- Long press support ---
    bool        _longPressEnable ;                                  // if true, long-press detection is enabled
    uint32_t    _longPressDurationMs;                               // duration in ms to consider a long press
    uint32_t    _pressConfirmTimeUs ;                               // timestamp when press was confirmed (in microseconds) since start
    bool        _longPressFiredThisDebounceSession ;                // to ensure long-press callback fires only once per press
    bool        _doublePressFiredThisSession ;                      // to ensure that not short Press callback are fire if a double-press is detected
   
    // ================================

    // === DOUBLE PRESS DETECTION API ===
    bool        _doublePressEnable;                     // if true, double-press detection is enabled
    uint32_t    _doublePressMaxWindowMs;                // max time window in ms to consider a double press
    uint32_t    _lastShortReleaseTimeUs;                // timestamp of last short release (in microseconds) since start
    bool        _waitingForSecondPress;                 // true if we are waiting for the second press in a double-press sequence

    // ================================

    // === SHORT PRESS DETECTION API ===
    bool        _pendingShortPress;                     // To avoid short Press callbacks to be trigger while a possible double press is possible on the first release

    // --- helper: fire all registered callbacks for a confirmed press ---
    void fireAll();                                     // fire all normal press callbacks
    void fireAllLongPress(uint32_t holdDurationMs);     // fire all long-press callbacks
    void fireAllDoublePress();                          // fire all double-press callbacks
   
    // Zero out the buffer and reset head to 0
    void clearBuffer();

   
};