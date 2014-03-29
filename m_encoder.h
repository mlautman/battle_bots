// -----------------------------------------------------------------------------
// functions to monitor two quadrature encoders
// version: 0.1
// date: Oct 24, 2012
// author: J. Fiene
// -----------------------------------------------------------------------------
// Attach the encoders as follows (reverse A and B to invert polarity)
// B0 = encoder 0, channel A
// B1 = encoder 0, channel B
// B2 = encoder 1, channel A
// B3 = encoder 1, channel B
//
// Calling m_encoder_init(buffer) to configure the lower-level operations
// will enable external interrupts on B0-3.  If you disable interrupts
// at a later time, you may miss encoder counts.
// -----------------------------------------------------------------------------

#ifndef m_encoder__
#define m_encoder__

#include "m_general.h"

// -----------------------------------------------------------------------------
// Public functions:
// -----------------------------------------------------------------------------

void m_encoder_init(long* buffer);
// FUNCTIONALITY:
// configure the m2 for two encoders
//
// TAKES:
// buffer: pointer to a two-element signed LONG array
//
// RETURNS:
// nothing

void m_encoder_zero(char channel);
// FUNCTIONALITY:
// reset the encoder counter
//
// TAKES:
// channel : 0 (first encoder), 1 (second encoder)
//
// RETURNS:
// nothing

#endif