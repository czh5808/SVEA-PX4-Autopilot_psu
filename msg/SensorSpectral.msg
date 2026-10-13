# Multispectral light sensor raw channel counts
#
# Channel order follows the AS7343 18-channel auto-SMUX read-out (see CHANNEL_* indices).

uint64 timestamp          # [us] Time since system start
uint64 timestamp_sample   # [us] Time the channel data was read from the sensor

uint32 device_id          # [-] Unique device ID for the sensor that does not change between power cycles

uint8 NUM_CHANNELS = 18

uint8 CHANNEL_FZ_450NM = 0    # Blue
uint8 CHANNEL_FY_555NM = 1    # Green (wide bandwidth)
uint8 CHANNEL_FXL_600NM = 2   # Orange
uint8 CHANNEL_NIR_855NM = 3   # Near infrared
uint8 CHANNEL_VIS_1 = 4       # Clear (cycle 1)
uint8 CHANNEL_FD_1 = 5        # Flicker detection (cycle 1)
uint8 CHANNEL_F2_425NM = 6    # Dark blue
uint8 CHANNEL_F3_475NM = 7    # Light blue
uint8 CHANNEL_F4_515NM = 8    # Blue-green
uint8 CHANNEL_F6_640NM = 9    # Brown
uint8 CHANNEL_VIS_2 = 10      # Clear (cycle 2)
uint8 CHANNEL_FD_2 = 11       # Flicker detection (cycle 2)
uint8 CHANNEL_F1_405NM = 12   # Violet
uint8 CHANNEL_F7_690NM = 13   # Red
uint8 CHANNEL_F8_745NM = 14   # Dark red
uint8 CHANNEL_F5_550NM = 15   # Green (narrow bandwidth)
uint8 CHANNEL_VIS_3 = 16      # Clear (cycle 3)
uint8 CHANNEL_FD_3 = 17       # Flicker detection (cycle 3)

uint16[18] counts         # [-] Raw ADC counts per channel
float32 gain              # [-] Analog gain applied to all channels
float32 integration_time  # [s] Integration time of each channel
bool saturated            # True if any channel saturated during this measurement
