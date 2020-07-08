#ifndef _CANLISTENER_H
#define _CANLISTENER_H
class CANListener
{
public:
    CANListener ();

    virtual bool frameHandler (CAN_message_t &frame, int mailbox, uint8_t controller);
    virtual void txHandler (int mailbox, uint8_t controller);

    void attachMBHandler (uint8_t mailBox);
    void detachMBHandler (uint8_t mailBox);
    void attachGeneralHandler (void);
    void detachGeneralHandler (void);

private:
    uint32_t callbacksActive; // bitfield indicating which callbacks are installed (for object oriented callbacks only)

    friend class FlexCAN_T4;     // class has to have access to the the guts of this one
};
#endif
