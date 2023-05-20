Can be run on any Raspberry Pi with a 64-bit OS and an attached Coral USB Accelerator.

To benchmark the "EfficientDet Lite 0" model (default) inference, run the command:

$ docker compose run --rm benchmark


To benchmark the "EfficientDet Lite 1" mode inference, run the command:

$ docker compose run --rm benchmark python3 ./app/detect_edl1.py


...while in the root of this directory.

DISCLAIMER: If Coral USB Accelerator has experienced a power off-on cycle it will throw an error on first inference due to its flashing process:
"ValueError: Failed to load delegate from libedgetpu.so.1.0"

Run it again and the Coral USB Accelerator should function as expected.