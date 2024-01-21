import usb_cdc      # type: ignore
import board        # type: ignore
import digitalio    # type: ignore
import storage      # type: ignore

# write access if the GP0 is connected to ground
write_access_pin = board.GP0
write_access_switch = digitalio.DigitalInOut(write_access_pin)
write_access_switch.direction = digitalio.Direction.INPUT
write_access_switch.pull = digitalio.Pull.UP
storage.remount("/", readonly=write_access_switch.value)

# enable serial coms for console + data
usb_cdc.enable(console=True, data=True)
