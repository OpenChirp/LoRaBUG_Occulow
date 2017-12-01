# First Update
```
git submodule update --init --recursive
```

# Occulow Application Overview
The entrypoint for the people counting application is in `Apps/sensorbugBasicDrivers/main.c`. It initializes two TI-RTOS tasks, the LoRaWan task and the people-counting task.

## Lora task
The LoRa task code is found in `main.c`.

### Provisioning
LoRa join values are found in `Commissioning.h`. By default, the device will use it's device-unique ID as its `DEV_EUI`. This can be changed by altering the `#define` `USE_BOARD_UNIQUE_ID_AS_DEV_EUI`. The application EUI and application key are hard-coded into the device. *The `APPLICATION_KEY` should be changed before flashing, and the key should not be kept in public version control*.

On boot, the device will check flash memory to see if it has successfully joined the network before. If it hasn't, it goes into a sleep state. Once the button is pressed, it will begin broadcasting its unique `DEV_EUI` in a BLE advertisement packet. The advertisement address is defined in `Services/ble_rf.c`. The `DEV_EUI` is broadcasted as an advertisement payload type `0x16`, "service data". The payload structure is `<length byte><payload type><payload data>`. The length byte gives the length of the structure, not including the length byte. So, the structure for these advertisement packets is `<0x09><0x16><8 byte dev EUI>`. After another button press, the device will attempt to join the network. Upon a successful join, it will write to flash memory so that it will join immediately on the next power cycle. Subsequent button presses cause the device to transmit the BLE advertisement packet for 10 seconds.

### Regular operation
After joining, every `APP_TX_DUTYCYCLE` milliseconds (default 30 seconds), the device will transmit a message described by the following protobuf structure (taken from `occulow.pb.h`):
```
typedef struct _CountMessage {
    uint32_t count_in;
    uint32_t count_out;
    uint32_t batteryVoltage;
    uint32_t batteryLevel;
} CountMessage;
```

`count_in` and `count_out` are the counts since the last send, and `batteryLevel` is a value from 0-255.

## People counting
The people counting task is found in `Services/pcService.c`. It reads frames from the GridEye, processes them, and attempts to count people moving in or out. After a period of no activity, the task will sleep and wake on an interrupt from the PIR sensor pin. Communication of counts between the tasks is controlled by a binary semaphore.

The people-counting code works by examining two trigger columns on either side of the frame. If a local maximum heat value above a threshold (`TRIGGER_THRESHOLD`) is found in a trigger column, the algorithm checks the adjacent columns in the past and future. If local maxima within a range of the original maximum are found in these columns, then we can count in either in or out.