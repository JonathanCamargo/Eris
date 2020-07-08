#include <FlexCAN_T4.h>
#include <canlistener.h>

CANListener::CANListener ()
{
    // none. Bitfield were bits 0-15 are the mailboxes and bit 31 is the general callback

    callbacksActive = 0;
}

/*
 * \brief Default CAN received frame handler.
 *
 * \param frame - CAN frame to process.
 * \param mailbox - mailbox number frame arrived at.
 * \param controller - controller number frame arrived from.
 *
 * \retval true if frame was handled, false otherwise.
 *
 */

bool CANListener::frameHandler (CAN_message_t &/*frame*/, int /*mailbox*/, uint8_t /*controller*/)
{

    /* default implementation that doesn't handle frames */

    return (false);
}

/*
 * \brief Default CAN transmission completed handler.
 *
 * \param mailbox - transmit mailbox that is now available.
 * \param controller - controller number.
 *
 */

void CANListener::txHandler (int /*mailbox*/, uint8_t /*controller*/)
{

}

/*
 * \brief Indicate mailbox has an active callback.
 *
 * \param mailBox - mailbox number.
 *
 * \retval None.
 *
 */

void CANListener::attachMBHandler (uint8_t mailBox)
{
    if ( (mailBox < NUM_MAILBOXES) ) {
        callbacksActive |= (1L << mailBox);
    }
}

/*
 * \brief Clear callback indicator for a mailbox.
 *
 * \param mailBox - mailbox number.
 *
 * \retval None.
 *
 */

void CANListener::detachMBHandler (uint8_t mailBox)
{
    if ( (mailBox < NUM_MAILBOXES) ) {
        callbacksActive &= ~(1UL << mailBox);
    }
}

/*
 * \brief Set general purpose callback indicator.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void CANListener::attachGeneralHandler (void)
{
    callbacksActive |= (1UL << 31);
}

/*
 * \brief Clear general purpose callback indicator.
 *
 * \param None.
 *
 * \retval None.
 *
 */

void CANListener::detachGeneralHandler (void)
{
    callbacksActive &= ~(1UL << 31);
}
