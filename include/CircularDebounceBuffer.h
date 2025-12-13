/**
 * @file CircularDebounceBuffer.h
 * @brief Header file for the CircularDebounceBuffer class, providing a robust debounce mechanism for digital button inputs.
 * Advanced interrupt-triggered, non-blocking button debouncer using a circular buffer and a consensus voting.
 *
 * This class implements a circular buffer-based debouncing algorithm for handling noisy button presses on micro-controllers like Arduino.
 * It collects periodic samples of the pin state and uses a consensus threshold to confirm presses and releases, making it resilient to bounce.
 * 
 *
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


/**
 * @brief Provides debounce modes for CircularDebounceBuffer.
 * 
 * @details
 * This enum defines the available debounce modes for the CircularDebounceBuffer class.
 */
namespace debounceMode
{
    enum class Mode{INTERRUPT_DRIVEN, POLLING};
};

// ========================

// Session timeout for gesture recognition.
static constexpr uint32_t GESTURE_SESSION_TIMEOUT_MS = 5000;                // 1.5 second of silence = new session

// Default long-press duration = 2000ms
static constexpr uint32_t LONG_PRESS_DURATION_MS_DEFAULT = 1000 ; 

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
 * @brief Arduino button debouncer — Allow Gesture detection for  short, long, and double-press button presses.
 *
 * Interrupt-triggered, non-blocking button handler with:
 * • Circular buffer + consensus threshold → immune to contact bounce
 * • Immediate long-press detection (fires during hold)
 * • Exclusive double-press (no duplicate short presses)
 * • Gesture session timeout → perfect behavior after silence
 * • Automatic pinMode() with internal pull-up
 * • Context-aware callbacks
 * • Zero false triggers, zero missed presses, zero ghost events
 *
 * @note Designed for mechanical switches with bounce < 20ms
 * @note Debouncing is armed only on FALLING edge → zero CPU usage when idle
 * @note Release uses symmetric inverted threshold
 * @note Constructor automatically sets pinMode():
 *       - INPUT_PULLUP (active-low, recommended)
 *       INPUT (active-high, external pull-down)
 *
 * @section Features
 * - Short press (on release, unless overridden by double/long)
 * - Long press (immediate at threshold, suppresses short press)
 * - Double press (exclusive — fires only double-press, never two shorts)
 * - Gesture reset after 1 second of silence → flawless transitions
 * 
 * Modes:
 *  - POLLING: no ISR needed, always samples in update()and debounce at the configured interval
 *  - INTERRUPT_DRIVEN: call startDebounce() from raw edge ISR , then samples in update() at the configured interval when debouncing
 *
 * @section Usage
 * @code{.cpp}
 * #include <Arduino.h>
 * #include "CircularDebounceBuffer.h"
 * #include <stdarg.h>
 *
 *
 * static constexpr uint8_t BUTTON_PIN = 2;    // Arduino Nano pin interrupts (2,3)
 * static constexpr uint8_t BUTTON2_PIN = 12;  // Arduino Nano pin
 *
 * CircularDebounceBuffer btn  = CircularDebounceBuffer::CreateInterruptDrive(0,BUTTON_PIN, true, 1000); 
 * CircularDebounceBuffer btn2 = CircularDebounceBuffer::CreatePolling(1, BUTTON2_PIN, true,1000);                                  // PIN 12, active-low, polling mode 
 *   
 *
 *
 * void serialPrintf(const char* fmt, ...)
 * {
 *  char buf[64];              // adjust size, beware of RAM limits on AVR
 *  va_list args;
 *  va_start(args, fmt);
 *  vsnprintf(buf, sizeof(buf), fmt, args);
 *  va_end(args);
 *  Serial.print(buf);
 * }
 *
 *
 *
 * void setup() 
 * {
 *  Serial.begin(115200);
 *  while(!Serial);
 *
 *
 *    
 *  btn.addCallback([] { Serial.println("\nSHORT PRESS!!!\n"); });
 *  btn.enableLongPress(2000);
 *  btn.addLongPressCallback([](uint32_t ms) { serialPrintf("\nLONG PRESS!!! : %lums\n", ms); });
 *  btn.enableDoublePress(400);
 *  btn.addDoublePressCallback([] { Serial.println("\nDOUBLE PRESS!!!\n"); });  
 *  
 *  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), [] { btn.startDebounce(); }, FALLING);
 * 
 *
 *  // Polling
 *  btn2.addCallback([]{Serial.println("\nSHORT PRESS!!!\n"); });
 *
 * 
 * }
 * 
 * void loop()
 * {
 *  btn.update();
 *  btn2.update();
 * }
 * @endcode
 *
 * @section Behavior
 * | Action                     | Output                     | Notes                     |
 * |----------------------------|----------------------------|---------------------------|
 * | Single short press         | SHORT PRESS                | After release             |
 * | Two fast presses (<400ms)  | DOUBLE PRESS               | Only — no short presses   |
 * | Long hold (>1000ms)        | LONG PRESS: XXXXms         | Immediate, no short press |
 * | Any transition             | Perfect                    | Always works              |
 *
 * @author David Soriano Artal
 * @date 2025
 * @version 1.0.0
 * @license MIT
 *
 * @example examples/AllFeatures/AllFeatures.ino
 * @example examples/Basic/Basic.ino
 * @example examples/LongPress/LongPress.ino
 * @example examples/DoublePress/DoublePress.ino
 */
class CircularDebounceBuffer {
public:

    //                           === FACTORY METHODS ===

    /**
     * @brief Factory method to create a CircularDebounceBuffer in POLLING mode.
     * 
     * @param id                An arbitrary ID (not used by zero‐arg callbacks)
     * @param pin               The Arduino digital pin to debounce
     * @param isActiveLow       If true, a LOW read means “pressed”
     * @param delayBetweenUs    How many microseconds between each raw sample
     * @return CircularDebounceBuffer instance in POLLING mode
     */
    static CircularDebounceBuffer CreatePolling(
        uint8_t id,
        uint8_t pin,
        bool    isActiveLow = true,
        uint32_t delayBetweenUs = 1000
    )
    {
        return CircularDebounceBuffer(id, pin, isActiveLow, delayBetweenUs, debounceMode::Mode::POLLING);
    }

    /**
     * @brief Factory method to create a CircularDebounceBuffer in INTERRUPT_DRIVEN mode.
     * 
     * @param id                An arbitrary ID (not used by zero‐arg callbacks)
     * @param pin               The Arduino digital pin to debounce
     * @param isActiveLow       If true, a LOW read means “pressed”
     * @param delayBetweenUs    How many microseconds between each raw sample
     * @return CircularDebounceBuffer instance in INTERRUPT_DRIVEN mode
     */
    static CircularDebounceBuffer CreateInterruptDriven(
        uint8_t id,
        uint8_t pin,
        bool    isActiveLow = true,
        uint32_t delayBetweenUs = 1000
    )
    {
        return CircularDebounceBuffer(id, pin, isActiveLow, delayBetweenUs, debounceMode::Mode::INTERRUPT_DRIVEN);
    }

    // ==================================================================================
    
    
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
        uint32_t delayBetweenUs = 1000,
        debounceMode::Mode mode = debounceMode::Mode::INTERRUPT_DRIVEN
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

    // === DETECTION MODE API ===
    debounceMode::Mode _debounceMode;       // Debounce mode (default to INTERRUPT_DRIVEN)

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