#include <HardwareSerial.h>
#include <Serial_CAN_FD.h>
#include <obd.h>
#include <DebugLog.h>

unsigned long __id = 0;                // can id
unsigned char __ext = 0;               // extended frame or standard frame
unsigned char __rtr = 0;               // remote frame or data frame
unsigned char __fdf = 0;               // can fd or can 2.0
unsigned char __len = 0;               // data length
unsigned char __dta[CAN_DATA_BUF_LEN]; // data buffer (CAN layer)

int canReceiveFrameBlocking(
    unsigned long *id,
    unsigned char *ext,
    unsigned char *rtr,
    unsigned char *fdf,
    unsigned char *len,
    unsigned char *dta,
    unsigned long timeout_ms = -1)
{
    unsigned long start = millis();
    int res = 0;
    while (millis() - start < timeout_ms)
    {
        // busy wait until timeout or read_can returns successful read
        res = read_can(id, ext, rtr, fdf, len, dta);

        if (res)
        {
            return res;
        }
    }
    LOG_DEBUG(" timeout waiting for frame");
    return 0;
}

int obdReceiveBlocking(
    unsigned long id,
    unsigned char *len,
    unsigned char *buf,
    unsigned long timeout_ms = -1)
{
    // does blocking reads until
    // * Frames with target id (id + 0x08) are received (including consecutive);
    //      returning length stored to buf
    // * timeout_ms ms elapses since calling; returning -1
    // * expected frame type mismatch; returning -2
    // * frame missed; returning -3

    int res = 0;
    unsigned short obd_len = 0;
    unsigned char frame_type = 0;
    unsigned long start = millis();
    unsigned long elapsed;

    *len = 0;

    // get first frame
    LOG_DEBUG("Receiving first frame... ");
    elapsed = millis();
    res = canReceiveFrameBlocking(
        &__id,
        &__ext,
        &__rtr,
        &__fdf,
        &__len,
        __dta,
        timeout_ms - (elapsed - start));

    // if we get a timeout then return
    if (res == 0)
        return -1;

    // OBD-II https://en.wikipedia.org/wiki/ISO_15765-2

    // analyze first frame
    frame_type = (__dta[0] & OBD_FRAME_BITMASK) >> 4;

    if (frame_type == 0x00)
    { // Single
        obd_len = (__dta[0] & OBD_LEN_BITMASK);
        memcpy(buf, &__dta[1], obd_len);
    }
    else if (frame_type == 0x01)
    {
        // First
        unsigned short rem_len = 0;
        // WARNING not portable check endianness of system!!
        obd_len = ((__dta[0] & OBD_LEN_BITMASK) << 8) | __dta[1];
        rem_len = obd_len - 6;

        // load first frame contents into obd buffer and advance
        memcpy(buf, &__dta[2], __len - 2);

        // let the transmitter know that it is OK to send remaining frames
        // OBD flow frame 0x30 0x00 0x0A (0x0A ms delay between consecutive)
        LOG_DEBUG("Sending flow control for remaining", rem_len, "bytes...");
        unsigned char tmp[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        unsigned short frame_num = 1;
        can_send(0x7E0, 0, 0, 0, 8, tmp);

        // begin receiving until a timeout or all frames received
        do
        {
            elapsed = millis();
            res = canReceiveFrameBlocking(
                &__id,
                &__ext,
                &__rtr,
                &__fdf,
                &__len,
                __dta,
                timeout_ms - (elapsed - start));

            if (res)
            {
                // WARNING not portable check endianness of system!!
                frame_type = (__dta[0] & OBD_FRAME_BITMASK) >> 4;
                unsigned char frame_counter = __dta[0] & OBD_LEN_BITMASK;

                // TODO frame_type should be 0x02, if it is 0x01, 0x03-0x04 then
                // there is a problem!
                if (frame_type != 2)
                    return -2;

                // track if frame counter is correct (if not, frames may be missed)
                // TODO send abort flow control if frame missed
                if (frame_num % 0x10 != frame_counter)
                    return -3;

                // frame size should also be 8 per OBD spec

                // copy bytes (frame 1 -> buf + 6, 1 -> buf + 13, ...)
                if (rem_len > 7)
                {
                    memcpy(buf + (6 + (7 * (frame_num - 1))), &__dta[1], __len - 1);
                    rem_len -= (__len - 1);
                }
                else
                {
                    memcpy(buf + (6 + (7 * (frame_num - 1))), &__dta[1], rem_len);
                    rem_len = 0;
                }

                // update frame number
                frame_num++;
            }
            else
            {
                // timeout
                return -1;
            }
        } while (rem_len > 0);
    }
    else
    { // Consecutive or Flow
        return -2;
    }

    *len = obd_len;
    return obd_len;
}

int obdRequestBlocking(
    unsigned long id,
    unsigned char *tmp,     // assumes that the caller allocates 8 bytes
    unsigned char *obd_len, // len of data from OBD II
    unsigned char *obd_dta, // data from OBD II
    unsigned long timeout = 1000)
{
    int res = 0;

    // id, extended/standard frame, remote/data frame, can fd/2.0, data len, buf
    can_send(id, 0, 0, 0, 8, tmp);

    // blocking receive
    res = obdReceiveBlocking(CAN_VIN_ID, obd_len, obd_dta, timeout);
    if (res > 0)
    {
        LOG_DEBUG("Received", *obd_len, "bytes");
    }
    else
    {
        LOG_DEBUG("Error receiving, code", res);
    }

    return res;
}