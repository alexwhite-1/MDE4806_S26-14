//Author: Alex White, 2/2/2026
//Version: 1
//This file is the header file for the quadrature decoder functionality. See other file for description.
#ifndef QUADRATURE_DECODER_HPP
#define QUADRATURE_DECODER_HPP

#include "quadrature_output.hpp"

/**
 * @class QuadratureDecoder
 * @brief Decodes quadrature encoder signals back to angle measurements
 * 
 * Processes A/B channel signals and index pulses to reconstruct the original
 * angle position. Supports both single and dual-axis decoding.
 */
class QuadratureDecoder {
public:
    // ========================================================================
    // Constructor & Configuration
    // ========================================================================
    
    /**
     * @brief Construct a quadrature decoder
     * @param cpr Counts per revolution for the encoder
     * @param num_axes Number of axes to decode (1 or 2)
     */
    QuadratureDecoder(int cpr = 4096, int num_axes = 1);
    
    /**
     * @brief Set CPR for all axes
     * @param cpr Counts per revolution
     */
    void setCPR(int cpr);
    
    /**
     * @brief Set CPR for a specific axis
     * @param axis Axis index (0 or 1)
     * @param cpr Counts per revolution
     */
    void setCPRAxis(int axis, int cpr);
    
    /**
     * @brief Get CPR for a specific axis
     * @param axis Axis index (0 or 1)
     * @return Counts per revolution
     */
    int getCPR(int axis) const;
    
    /**
     * @brief Set number of axes
     * @param num_axes 1 or 2
     */
    void setNumAxes(int num_axes);
    
    /**
     * @brief Get number of axes
     * @return Number of axes (1 or 2)
     */
    int getNumAxes() const;
    
    // ========================================================================
    // Decoder State Management
    // ========================================================================
    
    /**
     * @brief Initialize decoder at a known index position
     * Assumes inputs are at the index position (A=0, B=0, Index=1)
     * @param axis Axis index (0 or 1)
     */
    void initializeAtIndex(int axis);
    
    /**
     * @brief Reset decoder state
     * @param axis Axis index (0 or 1)
     */
    void reset(int axis);
    
    /**
     * @brief Check if decoder is synchronized
     * @param axis Axis index (0 or 1)
     * @return true if synchronized to index
     */
    bool isSynchronized(int axis) const;
    
    // ========================================================================
    // Signal Input Processing
    // ========================================================================
    
    /**
     * @brief Process quadrature signals from QuadratureOutput
     * @param output Reference to QuadratureOutput object
     */
    void processPulse(const QuadratureOutput& output);
    
    /**
     * @brief Process quadrature signals manually (single axis)
     * @param axis Axis index
     * @param channel_a A channel signal (0 or 1)
     * @param channel_b B channel signal (0 or 1)
     * @param index_signal Index signal (0 or 1)
     */
    void processPulse(int axis, int channel_a, int channel_b, int index_signal);
    
    /**
     * @brief Process quadrature signals manually (dual axis)
     * @param channel_a_1 Axis 1 A channel
     * @param channel_b_1 Axis 1 B channel
     * @param channel_a_2 Axis 2 A channel
     * @param channel_b_2 Axis 2 B channel
     * @param index_signal Index signal (shared)
     */
    void processPulse(int channel_a_1, int channel_b_1, 
                      int channel_a_2, int channel_b_2, int index_signal);
    
    // ========================================================================
    // State Query
    // ========================================================================
    
    /**
     * @brief Get current position count (in encoder steps)
     * @param axis Axis index (0 or 1)
     * @return Position count
     */
    int getPositionCount(int axis) const;
    
    /**
     * @brief Get current angle in degrees
     * @param axis Axis index (0 or 1)
     * @return Angle in degrees [0, 360)
     */
    double getAngle(int axis) const;
    
    /**
     * @brief Get current angle in radians
     * @param axis Axis index (0 or 1)
     * @return Angle in radians [0, 2π)
     */
    double getAngleRadians(int axis) const;
    
    /**
     * @brief Get absolute position count (may exceed full revolution)
     * @param axis Axis index (0 or 1)
     * @return Absolute position count
     */
    long long getAbsoluteCount(int axis) const;
    
    /**
     * @brief Get number of complete revolutions
     * @param axis Axis index (0 or 1)
     * @return Number of complete 360-degree revolutions
     */
    int getRevolutionCount(int axis) const;
    
    // ========================================================================
    // Raw Signal State
    // ========================================================================
    
    /**
     * @brief Get last A channel value
     * @param axis Axis index (0 or 1)
     * @return 0 or 1
     */
    int getLastChannelA(int axis) const;
    
    /**
     * @brief Get last B channel value
     * @param axis Axis index (0 or 1)
     * @return 0 or 1
     */
    int getLastChannelB(int axis) const;
    
    /**
     * @brief Get last index signal value
     * @param axis Axis index (0 or 1)
     * @return 0 or 1
     */
    int getLastIndex(int axis) const;
    
    // ========================================================================
    // Diagnostics & Debug
    // ========================================================================
    
    /**
     * @brief Get current quadrature state (0-3)
     * State 0: AB=00, State 1: AB=10, State 2: AB=11, State 3: AB=01
     * @param axis Axis index (0 or 1)
     * @return Current state (0-3)
     */
    int getQuadratureState(int axis) const;
    
    /**
     * @brief Get total pulses received on this axis
     * @param axis Axis index (0 or 1)
     * @return Pulse count
     */
    unsigned long long getPulseCount(int axis) const;
    
    /**
     * @brief Get index pulse count
     * @param axis Axis index (0 or 1)
     * @return Number of times index signal was detected
     */
    unsigned long long getIndexPulseCount(int axis) const;
    
    /**
     * @brief Get error count (invalid state transitions)
     * @param axis Axis index (0 or 1)
     * @return Count of invalid transitions
     */
    unsigned long long getErrorCount(int axis) const;
    
    /**
     * @brief Check for errors
     * @param axis Axis index (0 or 1)
     * @return true if any errors detected
     */
    bool hasErrors(int axis) const;
    
    /**
     * @brief Clear all accumulated state
     * @param axis Axis index (0 or 1)
     */
    void clearErrors(int axis);
    
    // ========================================================================
    // Formatted Output
    // ========================================================================
    
    /**
     * @brief Get formatted output as CSV string (single axis)
     * Format: "angle,position_count,revolutions"
     * @param axis Axis index (0 or 1)
     * @return CSV formatted string
     */
    std::string getFormattedOutput(int axis) const;
    
    /**
     * @brief Get formatted output for dual axis
     * Format: "angle1,position_count1,revolutions1,angle2,position_count2,revolutions2"
     * @return CSV formatted string
     */
    std::string getFormattedOutputDual() const;

private:
    // Configuration
    static const int MIN_CPR = 1;
    static const int MAX_CPR = 9000;
    static const int MAX_AXES = 2;
    
    int cpr_[MAX_AXES];
    int num_axes_;
    
    // Decoder state per axis
    struct AxisState {
        long long absolute_count;           // position count (can be negative)
        int last_channel_a;                 // last A signal (0 or 1)
        int last_channel_b;                 // last B signal (0 or 1)
        int last_index;                     // last index signal (0 or 1)
        bool synchronized;                  // synchronized to index
        unsigned long long pulse_count;     // total pulses processed
        unsigned long long index_pulse_count; // index pulses seen
        unsigned long long error_count;     // invalid transitions
    };
    
    AxisState state_[MAX_AXES];
    
    // Quadrature state machine
    static const int QUADRATURE_STATES = 4;
    static const int VALID_TRANSITIONS[QUADRATURE_STATES][QUADRATURE_STATES];
    
    /**
     * @brief Encode channel state to quadrature state (0-3)
     * @param a Channel A (0 or 1)
     * @param b Channel B (0 or 1)
     * @return State 0-3
     */
    static int encodeState(int a, int b);
    
    /**
     * @brief Determine direction from state transition
     * @param from Source state (0-3)
     * @param to Destination state (0-3)
     * @return +1 for forward, -1 for reverse, 0 for invalid
     */
    static int getDirection(int from, int to);
    
    /**
     * @brief Check if state transition is valid
     * @param from Source state (0-3)
     * @param to Destination state (0-3)
     * @return true if valid quadrature sequence
     */
    static bool isValidTransition(int from, int to);
    
    /**
     * @brief Process a single pulse on an axis
     * @param axis Axis index
     * @param a Channel A signal
     * @param b Channel B signal
     * @param index Index signal
     */
    void processAxisPulse_(int axis, int a, int b, int index);
    
    /**
     * @brief Clamp CPR to valid range
     * @param cpr Input CPR value
     * @return Clamped CPR value
     */
    static int clampCPR_(int cpr);
};

#endif // QUADRATURE_DECODER_HPP